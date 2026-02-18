## Team00000 Reference Module

`Team00000` is the mentor-maintained reference module for this multi-team FTC repository.
It is intentionally student-facing and demonstrates production-quality structure, naming,
telemetry, and control flow that teams can adapt in their own modules.

## Student Teams

This repository supports the following student competition teams:

- `Team12395` - Rattler Robotics
- `Team12397` - High Voltage Rattlers
- `Team13580` - Tech Serpents
- `Team13581` - Fang Gang
- `Team13588` - Rattlerbots
- `Team13590` - Viperstrike

## Module Roles

| Module | Purpose |
|---|---|
| `Team00000` | Mentor reference implementation and sample patterns. |
| `Team12395` | Team-specific robot code. |
| `Team12397` | Team-specific robot code. |
| `Team13580` | Team-specific robot code. |
| `Team13581` | Team-specific robot code. |
| `Team13588` | Team-specific robot code. |
| `Team13590` | Team-specific robot code. |
| `TeamShared` | Shared utilities and abstractions used across teams. |

## How Students Should Use Team00000

1. Start by reading one reference OpMode end-to-end before copying any code.
2. Copy only the patterns your robot needs (for example: drive state machine, AprilTag assist, telemetry layout).
3. Move reusable logic into your own team module or `TeamShared`.
4. Keep your team module names and hardware mappings aligned with your robot configuration.

## Professional Expectations

Code in `Team00000` should model:

- Clear intent through naming and concise comments.
- Predictable control flow and explicit safety defaults.
- Reproducible tuning via constants and dashboard-backed parameters.
- Telemetry that communicates state, not just raw numbers.
- Consistent formatting and maintainable structure suitable for code review.
