# Amendment 2b (addendum to amendment 2; registered before any run on cell 0.2.2)

Amendment 2 (`amendment_2.md`, sha256 f7ee1d8a9c7cf286e41c734f589d0737acabcf0c9a8b14a8c91f7841b5fbc623) is applied with
these changes:

1. Item 6 (S5, ship the survey photos once) is not applied. The engine builds the task image with a .dockerignore that
   leaves environment/setup/ out of the build context (`COPY setup/payload/given/survey` fails: ale validate exit 3,
   "excluded by .dockerignore"), and the domain lint requires tests/setup_payload/ to be a byte mirror of
   environment/setup/payload/ (30 problems without the survey). The survey photos stay in both places, byte-identical,
   as in the benchmark's other tasks; environment/Dockerfile, tests/test.sh and tests/review/sync_payload.py keep the
   frozen package's bytes.
2. Item 4, two accuracy corrections of the drafted text:
   - the joint-mode lag per joint at full stiffness reads "0.02-0.03 s for joints 1, 3 and 5-7, 0.08 s for joint 4,
     0.1 s for joint 2" (min(0.1, cap / kp) per joint: 0.020, 0.100, 0.021, 0.084, 0.028, 0.031, 0.022 s);
   - the F/T sign sentence was checked on the cell: with the hands pointing straight down at rest and untared, force_N
     along the hand's z read -7.3 and -8.7 N (left, right; the hand's weight plus the drawn bias), i.e. about 7.5 N
     along the hand's -z, as the sentence says.

Nothing else changes: items 1-5, 7 and 8 as registered; the hidden set, full_at and the anchor unchanged. The
agent-visible files then differ from c892e1b7 in catalog.md, cell_manual.md, describe_example.json and instruction.md
only.
