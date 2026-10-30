# EX262: the idle 802.15.4 radio's round tick stops after the scene

Base: main `2e36224d`. Code commit: `69e441e65efb96c8c42df9dabb8870107cc5e670`; this receipt records that code. Toolchain 1.99.0, release profile unchanged. ARM64 macOS host.

Branch rebased onto main `66eb9704`; code range-diff is `=`.

## What

The C6 ticks the IEEE 802.15.4 radio (`Ieee802154::tick`, CPU clock domain) every scheduling round, whatever the firmware does. After the scene step (the 100 ms channel-level drift, which stays every round) the tick tested the energy-scan countdown, seven countdowns (`tx_sfd`, `rx_sfd`, `tx_done`, `rx_done`, `ack_timeout`, `ack_turnaround`, `ack_done`) and both radio timers. With none of them armed each of these is a no-op.

Profile of main (macOS `sample`, 5 s, top of stack, [profile-summary.json](profile-summary.json)): `Bus::tick` (into which the radio tick is inlined) is the top self-time symbol on c6-hello (32.8%) and c6-contiki (22.6%). An attribution probe of main with the radio tick and its neighbours forced out of line (probe only, not committed) puts `Ieee802154::tick` alone at 7.8% of samples on c6-hello and 6.2% on c6-contiki.

Now a `live` flag:

- is set by every entry that can arm something: a register write (`Device::write`: commands, timer and ED registers), `receive` and `tx_loaded`;
- is recomputed at the end of a full tick: `ed_left.is_some()`, either timer running, or any of the seven countdowns armed;
- when clear, the tick returns right after the scene step only if the publicly writable `ed_left` is also empty.

Public `ed_left` can be armed without an entry point; the idle gate checks it directly. The remaining publicly writable fields affect results but do not independently arm the private timers or countdowns. The scene, events, deadlines (`next_deadline` unchanged) and every state transition are the same. Not touched by the open C6 speed changes (levers 2–7, the LEDC timer skip and the inline fetch/decode change): production changes are confined to `esp32c6/src/radio.rs`.

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

The candidate `esp32sim-c6` built from `69e441e6` (`cargo +1.99.0 build --release --bins`) has SHA-256 `5791bbaa15cae8642960f10953d8675b42a376aac2764d915be7c67e955ff80e`, the binary measured here.

## Hot-path disassembly

`cg_lib.py` + `classify.py` ([codegen-classes.json](codegen-classes.json)): every text symbol of the four release binaries, normalized for addresses, data pages, import stubs and alignment nops. In each binary the same 10 symbols change code, all in the C6 crate or the 802.15.4 peer of the multi-node runner; 2 more differ only in a masked page offset. No Xtensa-chip or C3/P4 code changes.

| Symbol | Base insns | Candidate insns | Why |
| --- | ---: | ---: | --- |
| `<SocBus as Bus>::tick` (C6, inlines the radio tick) | 2158 | 2170 | the `live` and public energy-countdown tests after the scene step and the recompute at the end of a full tick |
| `SocBus::periph_write` (inlines `Ieee802154::write`) | 1939 | 1942 | `live = true` on a radio register write |
| `Peripherals::new` | 1716 | 1717 | the `false` initialiser |
| `cooja::run` (inlines `receive`) | 3308 | 3309 | `live = true` on a received frame |
| `Ieee802154::note_unknown`, `Ieee802154::finish_rx`, `Peripherals::read32`, `Peer::report_radio`, `SocBus::report`, `Dispatch::debug` | same | same | same opcode sequence; only `Ieee802154` field offsets, stack slots and register numbers differ (the struct gained a byte) |

On an idle radio the round tick now does the scene accumulate/compare, a live-flag and public energy-countdown check, instead of the scan test, seven countdown tests and two timer tests.

## Mutations

`mutate.py <scratch-worktree> <out>` runs the C6 library tests followed by integration tests when necessary ([mutations.json](mutations.json)). All 21 cases compiled: 15 assertion-killed, five equivalent survivors and the unmutated control.

| Mutant | Result | Evidence |
| --- | --- | --- |
| control-unmutated | passes | unmutated control |
| gate-removed | passes | equivalent: full tick as baseline |
| gate-inverted | killed | `deadline_follows_the_armed_countdowns`, `ar_transmit_waits_for_its_ack`, `a_late_ack_is_dropped_rather_than_reported_as_a_frame` |
| public-ed-ignored | killed | `public_energy_countdown_runs_after_idle` |
| write-sets-no-live | killed | `a_quiet_radio_ticks_only_the_scene_and_every_arming_entry_wakes_it`, `deadline_follows_the_armed_countdowns` |
| receive-sets-no-live | killed | `a_quiet_radio_ticks_only_the_scene_and_every_arming_entry_wakes_it` |
| tx-loaded-sets-no-live | killed | `a_quiet_radio_ticks_only_the_scene_and_every_arming_entry_wakes_it` |
| recompute-always-false | killed | `a_quiet_radio_ticks_only_the_scene_and_every_arming_entry_wakes_it`, `a_late_ack_is_dropped_rather_than_reported_as_a_frame`, `ar_transmit_times_out` |
| recompute-always-true | killed | `a_quiet_radio_ticks_only_the_scene_and_every_arming_entry_wakes_it` |
| recompute-no-ed | passes | equivalent: public countdown is checked by the gate directly |
| recompute-timer0-only | killed | `a_quiet_radio_ticks_only_the_scene_and_every_arming_entry_wakes_it` |
| recompute-no-timers | killed | `deadline_follows_the_armed_countdowns`, `a_quiet_radio_ticks_only_the_scene_and_every_arming_entry_wakes_it` |
| recompute-and | killed | `a_quiet_radio_ticks_only_the_scene_and_every_arming_entry_wakes_it`, `deadline_follows_the_armed_countdowns` |
| recompute-no-tx_sfd | passes | equivalent: tx_done remains armed whenever tx_sfd is armed |
| recompute-no-tx_done | killed | `a_quiet_radio_ticks_only_the_scene_and_every_arming_entry_wakes_it`, `tx_start_requests_the_frame_then_times_the_air` |
| recompute-no-rx_sfd | passes | equivalent: rx_done remains armed whenever rx_sfd is armed |
| recompute-no-rx_done | killed | `a_quiet_radio_ticks_only_the_scene_and_every_arming_entry_wakes_it`, `rx_lands_after_its_air_time` |
| recompute-no-ack_timeout | killed | `a_late_ack_is_dropped_rather_than_reported_as_a_frame`, `ar_transmit_times_out` |
| recompute-no-ack_turnaround | killed | `received_ar_frame_is_acknowledged` |
| recompute-no-ack_done | killed | `received_ar_frame_is_acknowledged` |
| new-live-true | passes | equivalent: first full tick recomputes |

The two SFD countdowns stay in the recompute so the flag does not depend on that pairing.

Public-API integration test `public_energy_countdown_runs_after_idle` checks a directly armed scan after an idle round. New test in `esp32c6/src/radio.rs`: `a_quiet_radio_ticks_only_the_scene_and_every_arming_entry_wakes_it` (after a quiet round, a received frame, a loaded TX frame, an energy-scan command and radio timer 1 each reopen the full tick; each event lands on its own cycle across 64-cycle rounds; the flag clears again once nothing is armed).

## Host instructions (not a CPU table)

Retired host instructions (`ir.py 10 <base> <candidate>`, `/usr/bin/time -l`, one run each), screening only, not a CPU comparison ([host-insns.json](host-insns.json)):

| Fixture | Base | Candidate | Change |
| --- | ---: | ---: | ---: |
| c6-hello | 17,959,596,448 | 17,484,231,812 | -2.65% |
| c6-contiki | 40,551,216,341 | 40,075,512,103 | -1.17% |
| c6-energy-scan | 232,639,183,787 | 232,175,324,054 | -0.20% |
| public-display-starter | 114,976,616,613 | 114,507,312,806 | -0.41% |

Same stop and console hash on every row; no CPU-time claim.

## CPU comparison

PENDING

## Limits

A firmware that keeps a radio timer or scan running (c6-energy-scan scans most of the time) pays the full tick as before plus the recompute; the change only removes work while the radio is idle. Native only, not a browser or hardware claim. CPU table (1 warmup + 7 pairs, quiet machine) pending.

## Checks

At `69e441e6`: `cargo +1.99.0 clippy --workspace --all-targets -- -D warnings` and the wasm32 `esp32sim-wasm --features jit-tests` clippy pass; `ESP32SIM_ROM_DIR=… cargo +1.99.0 test --release --workspace -- --include-ignored --skip external_` passed (empty `HOME`); plain `cargo +1.99.0 test --release --workspace` passed (empty `HOME`); `tools/wasm-build.sh` + `node tools/wasm-test.mjs hello c3-hello c6-hello c6-energy-scan c6-contiki c6-contiki-net c6-rpl-net panel` ok; `node tools/check-evidence-privacy.mjs` pass; `git diff --check` clean. No goldens changed.
