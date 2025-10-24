Renesas Device Tree Overlay Naming
==================================

Overview
--------
This document defines the naming convention for Renesas board device tree
overlays (.dtbo). The goal is to make overlay intent obvious and consistent
across boards and features.

Naming Convention
-----------------
- Pattern:

  - `${model_string}-${revision_major}.${revision_minor}-${feature}.dtbo`

- Allowed/used feature suffixes (current set):

  - `ext-i2c`
  - `ext-spi`
  - `can`
  - `dsi`
  - `ov5640`
  - `cru-csi-ov5645`

Definitions
-----------
- `model_string`: Short, lowercase board identifier, hyphen-delimited.
  Examples: `rzg2l-evk`, `rzg2l-sbc`, `rzv2l-evk`, `rzv2h-rdk`.
- `revision_major`/`revision_minor`: Hardware revision of the board, e.g. `1.0`.
- `feature`: Hyphen-delimited description of the interface or peripheral enabled
  by the overlay. Prefer common tokens and keep lowercase.

Source File Naming
------------------
- The overlay source (`.dts`/`.dtso`) uses the same base name as the compiled
  overlay (`.dtbo`). For example:

  - `rzg2l-sbc-1.0-ext-spi.dts` -> `rzg2l-sbc-1.0-ext-spi.dtbo`

Examples (Current Overlays)
---------------------------
- `rzg2l-evk-1.0-cru-csi-ov5645.dts`  -> `rzg2l-evk-1.0-cru-csi-ov5645.dtbo`
- `rzg2l-sbc-1.0-can.dts`             -> `rzg2l-sbc-1.0-can.dtbo`
- `rzg2l-sbc-1.0-dsi.dts`             -> `rzg2l-sbc-1.0-dsi.dtbo`
- `rzg2l-sbc-1.0-ext-i2c.dts`         -> `rzg2l-sbc-1.0-ext-i2c.dtbo`
- `rzg2l-sbc-1.0-ext-spi.dts`         -> `rzg2l-sbc-1.0-ext-spi.dtbo`
- `rzg2l-sbc-1.0-ov5640.dts`          -> `rzg2l-sbc-1.0-ov5640.dtbo`
- `rzv2h-rdk-1.0-can.dts`             -> `rzv2h-rdk-1.0-can.dtbo`
- `rzv2h-rdk-1.0-ext-spi.dts`         -> `rzv2h-rdk-1.0-ext-spi.dtbo`
- `rzv2l-evk-1.0-cru-csi-ov5645.dts`  -> `rzv2l-evk-1.0-cru-csi-ov5645.dtbo`

Guidelines
----------
- Use lowercase and hyphens. Avoid spaces and camelCase.
- Keep feature tokens short and reusable across boards.
- When variants are needed, append a qualifier at the end,
  e.g. `ext-i2c`, `ext-spi`.
- Ensure the `Makefile  <../../../../../arch/arm64/boot/dts/renesas/overlays/Makefile>`_ includes the matching `.dtbo` name.

Optional: Schema/Compatible Naming
----------------------------------
- For dt-schema bindings, prefer the overlay root compatible string to follow
  the same base name: `renesas,${model_string}-${revision_major}.${revision_minor}-${feature}`.
- Name the binding file after the first compatible, e.g.:
  `renesas,rzg2l-sbc-1.0-ext-spi.yaml`.
