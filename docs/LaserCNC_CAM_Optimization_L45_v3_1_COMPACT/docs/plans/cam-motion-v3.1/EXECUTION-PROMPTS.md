# Execution Prompts — High-Capability Model / Low-Overhead Mode

## Start or continue one batch

```text
Execute docs/plans/cam-motion-v3.1/Bx-*.md on the current local repository.

Authoritative context, in order:
1. docs/plans/cam-motion-v3.1/README.md
2. docs/plans/cam-motion-v3.1/EXECUTION-CONTRACT.md
3. the active Bx document
4. only the extra references listed for that batch in CONTEXT-MAP.md

Do not redo architecture selection. Do not load/read the entire planning package unless an ESCALATE condition requires it.

Execution style:
- spend the majority of effort coding, inspecting repository facts, debugging and testing;
- implement consecutive workstreams in the same batch without asking for acceptance between them;
- Quick/Local checks are health checks, not completion ceremonies;
- after a passing soft checkpoint, continue automatically;
- do not restate Goal/Facts/MUST lists in your response;
- do not create one evidence document per subtask;
- do not run full regression after every helper/file change;
- preserve unrelated user changes.

Only stop before batch completion for: ESCALATE, frozen-contract ambiguity, safety invariant violation, persistent test failure, or a repository delta that invalidates the batch design.

At batch exit:
- run the batch Stage Gate once;
- produce a concise stage summary with commit range/changed areas/tests/raw evidence/known limitations;
- update STATUS-MATRIX.md;
- stop and request Astra stage review.
```

## Continue after Astra PASS

```text
Astra passed the previous batch. Start the next batch using only the previous stage summary plus the frozen contracts. Do not re-review or re-test the full earlier batch unless this batch changes one of its public contracts or a regression implicates it.
```

## Handling Astra PASS_WITH_PATCH

```text
Apply only the review patches required to close the named issue(s). Run the smallest directly relevant regression. Do not reopen the full previous batch unless the patch changes its frozen contract. Then continue to the next batch.
```
