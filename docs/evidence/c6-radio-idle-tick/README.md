# EX259: the idle 802.15.4 radio's round tick stops after the scene

Base: main `2e36224d`. Code commit: `c03dcd9e`; this receipt is the commit after it. Toolchain 1.99.0, release profile unchanged. ARM64 macOS host.

## What

The C6 ticks the IEEE 802.15.4 radio (`Ieee802154::tick`, CPU clock domain) every scheduling round, whatever the firmware does. After the scene step (the 100 ms channel-level drift, which stays every round) the tick tested the energy-scan countdown, seven countdowns (`tx_sfd`, `rx_sfd`, `tx_done`, `rx_done`, `ack_timeout`, `ack_turnaround`, `ack_done`) and both radio timers. With none of them armed each of these is a no-op.

Profile of main (macOS `sample`, 5 s, top of stack, [profile-summary.json](profile-summary.json)): `Bus::tick` (into which the radio tick is inlined) is the top self-time symbol on c6-hello (32.8%) and c6-contiki (22.6%). An attribution probe of main with the radio tick and its neighbours forced out of line (probe only, not committed) puts `Ieee802154::tick` alone at 7.8% of samples on c6-hello and 6.2% on c6-contiki.

Now a `live` flag:

- is set by every entry that can arm something: a register write (`Device::write`: commands, timer and ED registers), `receive` and `tx_loaded`;
- is recomputed at the end of a full tick: `ed_left.is_some()`, either timer running, or any of the seven countdowns armed;
- when clear, the tick returns right after the scene step.

Nothing else arms a countdown, the scan or a timer, so a clear flag means all those steps are no-ops. The scene, events, deadlines (`next_deadline` unchanged) and every state transition are the same. Not touched by the open C6 speed changes (levers 2–7, the LEDC timer skip and the inline fetch/decode change): the diff is `esp32c6/src/radio.rs` only.

## Exact work

`exact.py <out> 10 <base> <candidate>`, 10 guest seconds, ROM boot: stop line (instructions, cycles, exceptions, interrupts), stderr with wall time removed and console SHA-256 identical on all six fixtures ([exact-work.json](exact-work.json), inputs in [inputs.json](inputs.json)). The C3 and P4 binaries link the changed crate, so they are checked too.

| Fixture | Stop | Console SHA-256 | Identical |
| --- | --- | --- | --- |
| c6-hello | 1600000000 insns, 1010 interrupts | d5d1f41f315b… | True |
| c6-contiki | 1600000000 insns, 12994 interrupts | 101f1060911f… | True |
| c6-energy-scan | 1600000000 insns, 28722 interrupts | 720c0e65fa18… | True |
| public-display-starter | 1600000000 insns, 10346 interrupts | 1e6395a895a9… | True |
| c3-hello | 1600000000 insns, 1009 interrupts | 6dd87f0ce7e0… | True |
| p4-hello | core0 3600000000 + core1 0 insns, 1011 interrupts | db6c8bcfa9ae… | True |

The candidate `esp32sim-c6` built from `c03dcd9e` (`cargo +1.99.0 build --release --bins`) has SHA-256 `62fd38d5…`, the binary measured here; rebuilt from the commit, same hash.

## Hot-path disassembly

`cg_lib.py` + `classify.py` ([codegen-classes.json](codegen-classes.json)): every text symbol of the four release binaries, normalized for addresses, data pages, import stubs and alignment nops. In each binary the same 10 symbols change code, all in the C6 crate or the 802.15.4 peer of the multi-node runner; 2 more differ only in a masked page offset. No Xtensa-chip or C3/P4 code changes.

| Symbol | Base insns | Candidate insns | Why |
| --- | ---: | ---: | --- |
| `<SocBus as Bus>::tick` (C6, inlines the radio tick) | 2158 | 2169 | the `live` test after the scene step (one byte load and a branch) and the recompute at the end of a full tick |
| `SocBus::periph_write` (inlines `Ieee802154::write`) | 1939 | 1942 | `live = true` on a radio register write |
| `Peripherals::new` | 1716 | 1717 | the `false` initialiser |
| `cooja::run` (inlines `receive`) | 3308 | 3309 | `live = true` on a received frame |
| `Ieee802154::note_unknown`, `Ieee802154::finish_rx`, `Peripherals::read32`, `Peer::report_radio`, `SocBus::report`, `Dispatch::debug` | same | same | same opcode sequence; only `Ieee802154` field offsets, stack slots and register numbers differ (the struct gained a byte) |

On an idle radio the round tick now does the scene accumulate/compare, one byte load and one taken branch, instead of the scan test, seven countdown tests and two timer tests.

## Mutations

`mutate.py <scratch-worktree> <out>`: each mutant edits one changed line of `esp32c6/src/radio.rs`; the esp32c6 library tests run first, then every esp32c6 integration test if none failed ([mutations.json](mutations.json)). The unmutated control passes. 15 of 20 mutants are killed by failing assertions (none by compile errors); the 4 survivors besides the control are equivalent:

| Mutant | Change | Result | Failing tests (first three) |
| --- | --- | --- | --- |
| control-unmutated | none | control: passes | |
| gate-removed | early return deleted | equivalent: passes | without the gate every step runs and is a no-op, as on main (speed only) |
| gate-inverted | `!self.live` → `self.live` | killed | a_quiet_radio_ticks_only_the_scene_and_every_arming_entry_wakes_it, a_late_ack_is_dropped_rather_than_reported_as_a_frame, received_ar_frame_is_acknowledged |
| write-sets-no-live | register write does not set `live` | killed | a_quiet_radio_ticks_only_the_scene_and_every_arming_entry_wakes_it, deadline_follows_the_armed_countdowns |
| receive-sets-no-live | `receive` does not set `live` | killed | a_quiet_radio_ticks_only_the_scene_and_every_arming_entry_wakes_it |
| tx-loaded-sets-no-live | `tx_loaded` does not set `live` | killed | a_quiet_radio_ticks_only_the_scene_and_every_arming_entry_wakes_it |
| recompute-always-false | recompute → `false` | killed | ar_transmit_times_out, a_quiet_radio_ticks_only_the_scene_and_every_arming_entry_wakes_it, a_late_ack_is_dropped_rather_than_reported_as_a_frame |
| recompute-always-true | recompute → `true` | killed | a_quiet_radio_ticks_only_the_scene_and_every_arming_entry_wakes_it |
| recompute-no-ed | scan countdown dropped | killed | a_quiet_radio_ticks_only_the_scene_and_every_arming_entry_wakes_it |
| recompute-timer0-only | timer 1 dropped | killed | a_quiet_radio_ticks_only_the_scene_and_every_arming_entry_wakes_it |
| recompute-no-timers | both timers dropped | killed | a_quiet_radio_ticks_only_the_scene_and_every_arming_entry_wakes_it, deadline_follows_the_armed_countdowns |
| recompute-and | first `\|\|` → `&&` | killed | a_quiet_radio_ticks_only_the_scene_and_every_arming_entry_wakes_it, deadline_follows_the_armed_countdowns |
| recompute-no-tx_sfd | `tx_sfd` dropped | equivalent: passes | `tx_sfd` is only armed by `tx_loaded`, together with the longer `tx_done`, and only cleared together with it (TX stop): `tx_sfd` armed implies `tx_done` armed |
| recompute-no-tx_done | `tx_done` dropped | killed | a_quiet_radio_ticks_only_the_scene_and_every_arming_entry_wakes_it, tx_start_requests_the_frame_then_times_the_air |
| recompute-no-rx_sfd | `rx_sfd` dropped | equivalent: passes | `receive` arms `rx_sfd` (`SFD air − elapsed`) only when it also arms `rx_done` (`full air − elapsed`, larger), and both are only cleared together: `rx_sfd` armed implies `rx_done` armed |
| recompute-no-rx_done | `rx_done` dropped | killed | a_quiet_radio_ticks_only_the_scene_and_every_arming_entry_wakes_it, rx_lands_after_its_air_time |
| recompute-no-ack_timeout | `ack_timeout` dropped | killed | a_late_ack_is_dropped_rather_than_reported_as_a_frame, ar_transmit_times_out |
| recompute-no-ack_turnaround | `ack_turnaround` dropped | killed | received_ar_frame_is_acknowledged |
| recompute-no-ack_done | `ack_done` dropped | killed | received_ar_frame_is_acknowledged |
| new-live-true | initialised `true` | equivalent: passes | the first full tick recomputes the flag |

The two SFD countdowns stay in the recompute so the flag does not depend on that pairing.

New test in `esp32c6/src/radio.rs`: `a_quiet_radio_ticks_only_the_scene_and_every_arming_entry_wakes_it` (after a quiet round, a received frame, a loaded TX frame, an energy-scan command and radio timer 1 each reopen the full tick; each event lands on its own cycle across 64-cycle rounds; the flag clears again once nothing is armed).

## Host instructions (not a CPU table)

Retired host instructions (`ir.py 10 <base> <candidate>`, `/usr/bin/time -l`, one run each; load-independent to about ±0.05%) ([host-insns.json](host-insns.json)):

| Fixture | Base | Candidate | Change |
| --- | ---: | ---: | ---: |
| c6-hello | 17,979,935,270 | 17,469,035,880 | -2.84% |
| c6-contiki | 40,607,826,557 | 40,084,389,565 | -1.29% |
| c6-energy-scan | 232,958,166,788 | 232,433,098,566 | -0.23% |
| public-display-starter | 115,169,860,777 | 114,623,097,789 | -0.47% |

Same work on every row (stop line and console hash equal to base). No wall-clock or CPU-time claim.

## Limits

A firmware that keeps a radio timer or scan running (c6-energy-scan scans most of the time) pays the full tick as before plus the recompute; the change only removes work while the radio is idle. Native only, not a browser or hardware claim. CPU table (1 warmup + 7 pairs, quiet machine) pending.

## Checks

At `c03dcd9e`: `cargo +1.99.0 clippy --workspace --all-targets -- -D warnings` and the wasm32 `esp32sim-wasm --features jit-tests` clippy pass; `ESP32SIM_ROM_DIR=… cargo +1.99.0 test --release --workspace -- --include-ignored --skip external_` 1391 passed, 0 failed (empty `HOME`); plain `cargo +1.99.0 test --release --workspace` 1330 passed, 0 failed, 91 ignored; `tools/wasm-build.sh` + `node tools/wasm-test.mjs hello c3-hello c6-hello c6-energy-scan c6-contiki c6-contiki-net c6-rpl-net panel` ok; `node tools/check-evidence-privacy.mjs` pass; `git diff --check` clean. No goldens changed.
