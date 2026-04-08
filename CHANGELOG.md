# CHANGELOG

All notable changes to this repository should be documented in this file.

## Entry format

For every change, add a section with:

- Date: `YYYY-MM-DD`
- Title
- Files changed
- Behavior impact
- Migration/runtime risk notes (if applicable)

Keep entries in reverse chronological order (newest first).

For active iterative work, prefer updating the current top entry instead of appending a new entry for each small adjustment.

---

## 2026-04-08 — Agent documentation and working-context setup

### Files changed

- [AGENTS.md](AGENTS.md)
- [CHANGELOG.md](CHANGELOG.md)

### Behavior impact

- No runtime code changes.
- Added repository architecture and data-flow guidance for future LLM/code agents.
- Added mandatory policy to record all future modifications in this changelog.
- Documented that current iteration priority should use:
	- [launch/run_lio_sam_ouster.launch.py](launch/run_lio_sam_ouster.launch.py)
	- [config/lio_sam_ouster.yaml](config/lio_sam_ouster.yaml)

### Migration/runtime risk

- None.
