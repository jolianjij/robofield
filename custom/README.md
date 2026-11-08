# Your own tasks, policies and systems

Everything in this folder is written by you (or starts as an example), and the RoboField app's
**Lab** tab edits, checks and runs it. Full guide: [docs/lab.md](../docs/lab.md).

| Folder | What goes in it | Where it is used |
|---|---|---|
| `tasks/` | Something a car should **learn**: start positions, what it sees, what earns reward, when it ends | Training tab: train a policy for it |
| `policies/` | A hand-written program that drives **one** car, 30 times a second | Lab tab: **Run** on a car |
| `systems/` | A program that drives **several** cars together | Lab tab: **Run** on several cars |

Examples to start from: `tasks/park_center.py`, `policies/patrol_square.py`,
`systems/follow_the_leader.py`. Deleted files go to `.deleted/`, not away.
