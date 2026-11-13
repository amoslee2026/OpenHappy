---
title: Snapshot assets
type: reference
purpose: guide
audience: both
direction: both
status: approved
version: 1.0.0
author: OpenHappy Team
section_meta: "@meta"
---

# Snapshot assets

Waveform screenshots referenced by `../../UserGuide.md` (currently the reference waves of `smoke_1wr1rd`).

## Naming convention

`<case>_<object>_<purpose>.png`, all lower case, underscore separated:

- `<case>`: test name, e.g. `smoke_1wr1rd`
- `<object>`: interface / module name, lower-case abbreviation, e.g. `axi` / `paif` / `cpif` / `lphy_ephy`
- `<purpose>`: `reference_wave` (reference waveform) and similar

## Current files (UserGuide 3.3.7 reference waves)

| File | Guide location | Content |
|---|---|---|
| `smoke_1wr1rd_axi_intf_reference_wave.png` | 3.3.7 figure 1 | AXI user interface (s0 request / m0 response) |
| `smoke_1wr1rd_paif_intf_reference_wave.png` | 3.3.7 figure 2 | PAIF interface (protocol layer <-> link layer, 512 bit) |
| `smoke_1wr1rd_cpif_intf_reference_wave.png` | 3.3.7 figure 3 | CPIF interface (link layer <-> logic_phy, 1024 bit) |
| `smoke_1wr1rd_lphy_ephy_intf_reference_wave.png` | 3.3.7 figure 4 | logic_phy main-band output (lane0, 17 x 64 bit) |

## Referencing

From `docs/UserGuide.md`:

```markdown
![AXI interface reference wave](assets/snapshot/smoke_1wr1rd_axi_intf_reference_wave.png)
```

## Regenerating

1. `cd verify && make run TEST=smoke_1wr1rd DUMP=1 TRAIN_BYPASS=1` produces `sim/smoke_1wr1rd/smoke_1wr1rd.vcd`
2. `make waves TEST=smoke_1wr1rd` (GTKWave), then select the interface signals of interest
3. GTKWave `File -> Print to File` (or a screenshot), named per the table above

## Guidelines

- Format `png`; keep each image <= 300 KB and the directory <= 10 MB
- Only final, reviewed images belong here; intermediate captures stay out
- Block diagrams do not go here: they live in `../../architecture/`
