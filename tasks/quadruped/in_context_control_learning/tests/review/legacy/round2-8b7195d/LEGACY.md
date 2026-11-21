# Legacy Round2 Snapshot

This is an inert, byte-for-byte archive of selected task paths at maintainer
commit `8b7195d`. It preserves the round2 image-only reference, privileged token
reference, source, registered selection and development/margin evidence. It is
not participant payload, current oracle code, current grader code or v4 evidence.
Do not execute its entrypoints against the current task. In particular its
unprotected `unlock` service must never be restored to the active runtime.

`task.toml` is named `task.toml.legacy` here, with unchanged bytes, so repository
task discovery cannot mistake this historical snapshot for a second active task.

The active seed/candidate files preserve the original contents and ordering.
The active v4 service has only observe, act and advance; its author reference
uses engine-staged private metadata rather than a privileged service endpoint.
