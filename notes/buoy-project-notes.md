# Buoy Project Notes

## Purpose

This file is now the notes index and workflow reference.

Use `notes/sessions/` for real dated meeting notes.
Create one file per day using the format `YYYY-MM-DD.md`.

## Canonical Research Files

The working research worksheets and completed research files are stored at:

`G:\.shortcut-targets-by-id\18-UlV7l8VkdaFUB_skdX38onmq8bF4nY\9 - Buoy Project\Research`

Check this location first when reviewing or importing completed subsystem research. The `docs/research/` directory in this repository may contain generated exports or earlier local copies rather than the latest completed worksheets.

## Session Files

- [2026-04-18](./sessions/2026-04-18.md)
- [2026-04-19](./sessions/2026-04-19.md)
- [2026-04-26](./sessions/2026-04-26.md)
- [2026-05-03](./sessions/2026-05-03.md)
- [2026-05-24](./sessions/2026-05-24.md)
- [2026-05-25](./sessions/2026-05-25.md)
- [2026-05-31](./sessions/2026-05-31.md)
- [2026-06-07](./sessions/2026-06-07.md)
- [2026-08-08](./sessions/2026-08-08.md)
- [2026-08-09](./sessions/2026-08-09.md)
- [2026-08-16](./sessions/2026-08-16.md)
- [2026-09-05](./sessions/2026-09-05.md)
- [2026-09-06](./sessions/2026-09-06.md)
- [2026-09-07](./sessions/2026-09-07.md)
- [2026-09-20](./sessions/2026-09-20.md)
- [2026-09-29](./sessions/2026-09-29.md)
- [2026-10-03](./sessions/2026-10-03.md)
- [2026-10-04](./sessions/2026-10-04.md)
- [2026-10-10](./sessions/2026-10-10.md)
- [2026-10-11](./sessions/2026-10-11.md)
- [2026-10-17](./sessions/2026-10-17.md)
- [2026-10-18](./sessions/2026-10-18.md)
- [2026-10-21](./sessions/2026-10-21.md)
- [2026-10-24](./sessions/2026-10-24.md)
- [2026-10-25](./sessions/2026-10-25.md)
- [2026-10-26](./sessions/2026-10-26.md)
- [2026-10-30](./sessions/2026-10-30.md)
- [2026-10-31](./sessions/2026-10-31.md)
- [2026-11-01](./sessions/2026-11-01.md)
- [2026-11-07](./sessions/2026-11-07.md)
- [2026-11-08](./sessions/2026-11-08.md)

## Note-Taking Rule

- For each real meeting or work session, write notes in a dated file inside `notes/sessions/`.
- Keep raw device logs, SD files, and conversation archives in `field-data/`; link from the dated meeting note. Meeting notes summarize decisions and outcomes.
- Append notes continuously during work, not only at session end.
- If multiple work blocks happen on the same day, keep them in the same dated file as separate sections.
- If one logical session continues across multiple days, keep the same session label in each new day file and add a continuation link to the prior day.
- Each dated file should include:
  - What We Did
  - Completed Tasks
  - Meeting Notes

## Multi-Day Continuation Pattern

- Use this section title style: `Session N (Continued from YYYY-MM-DD Session N)`.
- Add one line at the top of that section linking the previous continuation point.
- Keep unresolved checklist items open until they are completed in a later day file.

## File Template

Copy this into a new file such as `notes/sessions/YYYY-MM-DD.md`.

```md
# YYYY-MM-DD

## Session 1

### What We Did

- 

### Completed Tasks

- [ ] 

### Meeting Notes

- 

## Session 2 (Continued from YYYY-MM-DD Session 2)

Previous continuation point: `notes/sessions/YYYY-MM-DD.md` Session 2

### What We Did

- 

### Completed Tasks

- [ ] 

### Meeting Notes

- 
```

