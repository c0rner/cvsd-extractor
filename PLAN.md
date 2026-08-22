# Fix the Six Open `bug` Issues in Three Review Units

## Summary

Implement three sequential changes, based on `main`:

1. **Checked ROM parsing:** #7 and #12.
2. **Stateful sequence decoding:** #6, #8, and #9.
3. **Streaming reports:** #15.

Each unit remains independently testable and reviewable. Existing unrelated untracked files and the Dependabot-only branch commit are excluded.

## Key Changes

### 1. Checked ROM and table parsing — #7, #12

**Status: completed**

- Add focused `thiserror`-based `RomError` and `ProgramError` types for touched parsing paths; leave the full library-wide error conversion to #13.
- Make `read_be_u16`, ROM-address translation, range construction, and bank-offset arithmetic checked and fallible. Store the actual ROM length in `RomHeader`; validate banked `0x4000..=0xBFFF` and fixed `0xC000..=0xFFFF` mappings.
- Audit `wpc89.rs` and `sound_program.rs` so externally supplied ROM bytes are accessed only through checked reads or validated slices. Use `checked_add` for `CvsdEntry::offset + size`; retain its public fields for compatibility.
- Treat malformed declared data as fatal:
  - truncated dispatch tables;
  - invalid voice/program table pointers or descriptors;
  - invalid bank selectors, impossible CVSD ranges, and out-of-ROM records.
- Preserve documented sentinels: CVSD table pointers below `0x4000` terminate the table, and sub-window channel pointers denote unused handler-`0x01` slots. A descriptor or program explicitly referenced by a dispatch entry is never silently treated as a sentinel.
- Return `ProgramExtraction { programs, diagnostics }` from `extract_programs`. Unsupported handler IDs become `ProgramDiagnostic::UnsupportedHandler { command, handler_id, param }`; malformed data returns `Err` and no authoritative partial report.
- Keep sequence-local control-flow failures nonfatal to the whole ROM extraction, but expose them through the typed sequence status introduced below.

### 2. Stateful sequence decoder — #6, #8, #9

- Replace the raw file-position decoder state with a small checked `SequenceCursor` containing the active bank selector and 6809 address. Initialize it with `SYSTEM_BANK`; fixed-bank reads ignore the selector.
- Store cursor states on the call stack and use a visited-state set containing the cursor, current opcode, and call stack to terminate indirect/call cycles before the instruction limit.
- **#6:** Handle branch opcodes exhaustively:
  - `IndirectOpcodeLoad` reads its checked 16-bit target, fetches the opcode there, and continues at the following byte.
  - An empty-stack `SubroutineReturn` fetches the next opcode from the current cursor as the firmware does.
  - Invalid targets, cycles, call-depth overflow, and instruction limits produce an incomplete typed status, never `complete: true`.
- **#8:** Treat `SetBankSwitch`'s operand as the raw bank-register selector. Read that operand in the old bank, advance the logical address, switch banks, then fetch the embedded next opcode from the new bank. Reject selectors for unavailable chips/pages with an incomplete diagnostic; the `programs` command remains U18-only.
- **#9:** Replace opcode lookahead with explicit `OperandRule` metadata:
  - `0x0A`, `0x1E`, and `0x3A`: key-code bit 7 set selects one timing byte; clear selects two.
  - `0x0B`: always two timing bytes.
  - `0x35`: always one timing byte.
  - Correct total operand/embedded-opcode counts for `0x1E` and `0x3A`.
  - Any future variable rule without sufficient state returns `AmbiguousLength` rather than guessing.
- Replace `complete` plus free-form `truncation` with `DecodeStatus::{Complete, Incomplete(SequenceIssue)}`. Include address/bank context in relevant `SequenceIssue` variants.

### 3. Streaming reports and broken pipes — #15

- Add canonical writer APIs:
  - `write_programs(&ProgramExtraction, &RomHeader, &mut impl Write) -> io::Result<()>`
  - `write_program_summary(&Path, &mut impl Write) -> Result<(), ProgramReportError>`
- Write headers, programs, instructions, and diagnostics incrementally. Format instruction operands and channel lists directly without intermediate `Vec<String>` allocations.
- Retain `format_programs` and `summarise_programs` as compatibility conveniences implemented through the writer path; the CLI must not use them.
- Lock stdout in `main`, stream the report, and treat only `io::ErrorKind::BrokenPipe` during output as successful early termination. Propagate all other read, parse, and output errors.
- Preserve existing report formatting except for explicit unsupported-handler diagnostics, typed incomplete reasons, and instruction differences caused by corrected decoding.

## Public Interface Changes

- `read_be_u16` and `RomHeader` address-translation methods return `Result`.
- Add bank-aware `RomHeader::to_file_offset_in_bank`.
- `extract_programs` returns `ProgramExtraction` rather than `Vec<SoundProgram>`.
- Add `ProgramDiagnostic`, `DecodeStatus`, `SequenceIssue`, focused parser/report error enums, and the streaming writer functions.
- Keep unrelated `anyhow`-based WAV/extraction APIs unchanged for the separate #13 work.

## Test and Acceptance Plan

- Build synthetic in-memory ROM fixtures; do not add commercial ROM data.
- **#7/#12:** test the zero-filled 128 KiB reproduction, truncated words/dispatch tables/descriptors, addresses below `0x4000`, fixed/banked boundaries, invalid pages/selectors, overflowing `CvsdEntry` ranges, legitimate sentinels, unsupported-handler diagnostics, and fatal referenced-pointer corruption.
- **#6:** test indirect continuation, invalid targets, empty-stack returns, ordinary call/return, and cyclic indirect branches.
- **#8:** test a cross-bank sequence where the old-bank contiguous byte is deliberately different, plus invalid bank translation. Add a minimized sparse Twilight Zone-style regression fixture containing only the relevant bank/opcode trace and assert its decoded trace/checksum.
- **#9:** cover both key-code-bit forms of `0x0A`, `0x1E`, and `0x3A`; verify fixed two-byte `0x0B` and fixed one-byte `0x35` using sequences where lookahead would have chosen incorrectly.
- **#15:** compare a golden formatted report, use a writer that returns `BrokenPipe`, verify the CLI maps it to success, and verify other writer errors propagate.
- Run `cargo fmt --check`, `cargo test --all-targets`, and `cargo clippy --all-targets -- -D warnings`. Manually rerun the malformed-ROM and `programs | head -n 1` reproductions.

## Assumptions and Defaults

- Deliver the work as three sequential review units, with checked parsing landing before decoder changes and streaming reports last.
- Malformed declared ROM data is fatal; unsupported command handlers are nonfatal structured diagnostics.
- Only focused errors needed by these fixes are introduced. The broader public error migration remains tracked by #13.
- All currently known variable-length opcode forms are resolvable from firmware state. No opcode-validity heuristic remains.
- Control-flow errors make only the affected sequence incomplete so other valid programs can still be reported with explicit status.
- Existing report text remains stable except where diagnostics or corrected decoding intentionally change it.
