# DASHBOARD FRONTEND

## OVERVIEW
`dashboard/` is the operator UI: React + TypeScript + Vite talking to rosbridge over WebSocket and the API node over HTTP.

## WHERE TO LOOK
| Task | Location | Notes |
|------|----------|-------|
| Main app wiring | `src/App.tsx` | rosbridge connection, REST calls, joystick, telemetry UI |
| Styling | `src/App.css` | layout + panel styling for the single-page UI |
| Runtime deps | `package.json` | `roslib`, `nipplejs`, Vite commands |
| Build config | `vite.config.ts` | minimal React/Vite setup |
| TS config roots | `tsconfig.json`, `tsconfig.app.json`, `tsconfig.node.json` | project references |

## CONVENTIONS
- `src/App.tsx` is currently the integration hub: ROS topics, REST commands, reconnect handling, and joystick publishing all live there.
- Frontend runtime assumes rosbridge at `ws://localhost:9090` and API at `http://localhost:8000` unless you deliberately parameterize both ends together.
- Live state is a hybrid flow: initial snapshot via `/api/state`, ongoing updates via ROS topics from `roslib`.
- `/cmd_vel` publishing is UI-driven and gated by connection + locomotion state; preserve that guard if you refactor controls.
- Joint display assumes the project-wide 29-joint layout grouped as legs, waist, and arms.

## ENTRY POINTS
- `npm run dev` - local Vite dev server.
- `npm run build` - typecheck + production build.
- `src/App.tsx` - main runtime entry for dashboard behavior.

## ANTI-PATTERNS
- Do not change rosbridge topic names or message expectations in the UI without matching ROS-side changes.
- Do not duplicate backend state logic in the UI when `/api/state` or ROS topics already provide it.
- Do not treat `dashboard/README.md` as authoritative project guidance; it is still mostly the stock Vite template.
- Do not commit `node_modules/` or `dist/` output.
- Do not silently widen `App.tsx` further; if you add major UI features, prefer extracting focused components/hooks.

## COMMANDS
```bash
cd /home/plate/unitree-g1-dashboard/dashboard && npm install
cd /home/plate/unitree-g1-dashboard/dashboard && npm run dev
cd /home/plate/unitree-g1-dashboard/dashboard && npm run build
cd /home/plate/unitree-g1-dashboard/dashboard && npm run lint
```

## NOTES
- There is no dedicated dashboard test suite yet; manual verification currently depends on the ROS/API stack being up.
- The frontend is substantial enough to deserve local documentation even though it is still mostly a single-page app.
