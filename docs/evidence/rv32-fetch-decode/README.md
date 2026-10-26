# EX249: C6 instruction fetch and RV32 decode inline into the step

Base: main `ddf753a5`. Toolchain 1.99.0, release profile unchanged. ARM64 macOS host.

## What

The C3/C6/P4 core is a plain interpreter. On the C6, every executed instruction called `SocBus::fetch` (out of line, four bounds-checked byte loads merged into the result) and then `decode32` or `decode_compressed` (out of line, the `Insn` returned through the stack). On a 5 s `sample` of c6-energy-scan on base these three were the top-of-stack for 48.9% of main-thread samples (`decode_compressed` 21.8%, `fetch` 15.7%, `decode32` 10.9%; local profile, not committed).

Two changes, no new state:

1. C6 `Bus::fetch` is `#[inline(always)]`, and a window that lies entirely inside its buffer is read as one 4-byte slice. A window cut short by the end of SRAM, ROM, LP SRAM or a mapped flash page takes the old byte loop, so the bytes past the end still read as zero. Resolution through the MMU is unchanged.
2. `decode32` and `decode_compressed` are `#[inline(always)]` into `decode_for`. This is the shared RV32 crate, so the C3 and P4 steps get the inlined decoders too (their fetch is unchanged).

The decoders, their results and the order of every bus access are unchanged.

## Exact work

`exact.py <out> 10 <base-probe> <candidate-probe>` with the local post-run retired-count report (`probe.py`, not committed), 10 guest seconds, ROM boot. Identical: retired instructions, scheduler work, interrupt totals and per-line sources, stderr (wall time removed) and console SHA-256 ([exact-work.json](exact-work.json)).

| Fixture | Retired | Interrupts | Console SHA-256 |
| --- | ---: | ---: | --- |
| c6-hello | 3,657,500 | 1,010 | d5d1f41f… |
| c6-contiki | 61,053,934 | 12,994 | 101f1060… |
| c6-energy-scan | 895,091,212 | 28,722 | 720c0e65… |
| public display starter | 373,050,392 | 10,346 | 1e6395a8… |
| c3-hello (`esp32sim-c3`) | – | 1,009 | 6dd87f0c… |
| p4-hello (`esp32sim-p4`) | – | 1,011 | db6c8bcf… |

C3 and P4 rows compare the unprobed release binaries (scheduler work, interrupts, stderr, console).

## Hot-path disassembly

`codegen.py <base-bins> <candidate-bins> codegen.json` over the four release binaries (esp32sim, -c3, -c6, -p4), 75 selected symbols each, normalized for addresses, data page offsets and import stubs ([codegen.json](codegen.json)). 69/75 identical in every binary. The six differences are the intended ones: the three RV32 `step_outcome` instantiations grow (C6 905 → 1832, C3 905 → 1721, P4 930 → 1753 instructions) and `decode32`, `decode_compressed` and the C6 `fetch` no longer exist as separate functions. Classic and S3 run loops, `step_blocks`, every bus tick, deadline and device function are byte-identical.

## Mutations

`mutate.py <scratch-worktree> <out>`: 6/6 killed by a failing test, none by compile errors ([mutations.json](mutations.json)). Fast window byte order, shifted window, last byte dropped, window clamped to the buffer, short window read as a whole one, and the zero fill of the tail all fail `bus::fetch_tests`. The inline attributes have no functional rule to mutate; the disassembly above records them.

## Daytime smoke (not a CPU table)

`smoke.py smoke.json 10 <main> <candidate>`: user CPU from `wait4`, 10 guest seconds, 1 warmup pair + 3 alternating pairs, 1-minute load 4.2–5.1 from unrelated work ([smoke.json](smoke.json)). A main-vs-main control ran right after it, load 2.8–4.1 ([smoke-aa.json](smoke-aa.json)).

| Fixture | Main median s | Candidate median s | Change | Candidate slower | A/A change |
| --- | ---: | ---: | ---: | ---: | ---: |
| c6-hello | 0.571 | 0.578 | +1.3% | 2/3 | −0.9% |
| c6-contiki | 1.492 | 1.476 | −1.1% | 1/3 | +0.5% |
| c6-energy-scan | 10.546 | 9.498 | −9.9% | 0/3 | +1.9% |
| public display starter | 3.657 | 3.271 | −10.6% | 0/3 | +0.4% |

The gain is on fixtures that execute instructions; the idle-heavy ones spend their time in idle rounds and stay within the A/A spread.

## Variants measured and not adopted

Rotating daytime comparisons (1 warmup + 3 rounds, loads 2–63, so only large differences mean anything; [explore.json](explore.json)). Names: `f1` one-slice fetch window only; `f2` plus `#[inline]` (fetch stayed out of line); `f3` plus `#[inline(always)]`; `d0` inlined decoders only; `f1d`, `f3d` combinations (`f3d` is this change, also `lever4`); `cand`/`m1024`/`m16384` a direct-mapped decode memo keyed by PC, fetched word and `misa.F` (4096/1024/16384 entries) instead of inlining; `f2m`, `f3m` fetch variants plus that memo.

The decode memo is not adopted. A local hit counter showed 90% hits on c6-energy-scan (76.6M misses by the 671M-th hit) and 99.5% on the starter, yet a hit cost nearly what decoding did: −6% on the starter, −3% to +3.5% on c6-energy-scan across runs, and nothing on top of `f3`. It also adds 112 KiB per core. Neither part alone matched `f3d` on c6-energy-scan in the same rounds.

## Checks

- `cargo +1.99.0 clippy --workspace --all-targets -- -D warnings`: pass.
- `cargo +1.99.0 clippy --release --target wasm32-unknown-unknown -p esp32sim-wasm --features jit-tests -- -D warnings`: pass.
- `ESP32SIM_ROM_DIR=<fw> cargo +1.99.0 test --release --workspace --no-fail-fast -- --include-ignored --skip external_` (empty HOME): 1323 passed, 0 failed.
- `cargo +1.99.0 test --release --workspace --no-fail-fast` (empty HOME, no ESP32SIM variables): 1270 passed, 0 failed, 76 ignored.
- `RUSTUP_TOOLCHAIN=1.99.0 tools/wasm-build.sh && node tools/wasm-test.mjs hello c3-hello c6-hello c6-energy-scan c6-contiki c6-contiki-net c6-rpl-net panel`: pass.
- `node tools/check-evidence-privacy.mjs`: pass.

Goldens unchanged.

## Limits

C3 and P4 are checked for exact work and disassembly, not timed. The display starter is a public C6 LCD starter image (hashes in [inputs.json](inputs.json)), not committed. Idle rounds, deadlines and the machine loop are untouched; the step itself (`step_outcome`, [EX168](../../experiments.md#ex168)'s step facts) is unchanged in source and only grows by the inlined fetch and decoders.

## CPU comparison

PENDING
