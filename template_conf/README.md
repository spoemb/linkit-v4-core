# template_conf/

Example DTE configuration files used as a starting point for typical deployments. Edit and push via PyLinkit (`pylinkit_cli config push <file>`) or over USB/BLE.

| File | Use case |
|------|----------|
| `turtle_tracker.cfg` | Sea turtle satellite tracker (Argos KIM2 / SMD) |
| `rspb_mortality.cfg` | RSPB bird mortality tracker (TPL5111 duty cycle, mortality detection) |

Each file is a sequence of `PARMW` / `CMDW` lines. The full list of parameters is in the [Parameters wiki page](https://github.com/arribada/linkit-v4-core/wiki/09-%E2%80%90-Parameters) and in [`../core/protocol/dte_params.cpp`](../core/protocol/dte_params.cpp).

When you change something here, bump the `FW_APP_VERSION` line at the top so the deployed device records which template was used.
