# Claude agent orientation

Setting up a fresh machine? **Follow `README.md` top to bottom** — it is
written for you, including NVIDIA driver install and the verify steps.

Working on the stack? Read in this order:
1. `README.md` — layout, build, the 9 critical nuances
2. `teleop/README.md` — operational manual (run procedure, safety, §6 how
   control actually works, §7 troubleshooting with refuted theories)
3. `docs/ARM.md` — driver/SDK internals, gripper compliance tuning (§6b)
4. `docs/HANDOFF.md` — most recent known state and unverified items

House rules learned the hard way:
- **Trust code over comments, and these docs over vendor docs.** Several
  vendor claims (rates, gravity model, "500 Hz") were false; §7 of the
  teleop README lists theories that were tested and REFUTED — do not
  re-chase them.
- `quest2ros/`, `ros_tcp_communication/`, and the vendor SDK carry local
  bug fixes with no upstream. Never "refresh" them from their origins.
- Both workspaces are symlink installs: edit Python → restart node. No
  rebuild unless files/entry points were added.
- The arm driver is dangerous-by-default: energizes on connect, no MIT
  velocity ceiling, motors latch on unclean shutdown. Never start/stop it
  casually, never run two, one SIGINT only.
- Never delete or auto-discard recorded demo episodes. Ever.
