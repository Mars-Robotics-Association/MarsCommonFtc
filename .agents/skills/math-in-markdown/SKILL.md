---
name: math-in-markdown
description: "ONLY for .md files. Apply correct math formatting conventions when writing or editing a markdown file that contains formulas or equations. Do NOT use for code comments, Java, or any non-markdown file."
---

Apply the following conventions when writing math in `.md` files.

## Delimiters

- Inline: `$...$`
- Display block: `$$...$$`

## Desmos compatibility

Use Desmos function names wherever a Desmos equivalent exists (e.g. `sign` not `sgn`). Desmos
uses `arctan(y, x)` as its two-argument atan2.

## GitHub rendering pitfalls

GitHub's markdown preprocessor intercepts `\` followed by any punctuation character before MathJax
sees it. Avoid `\,` `\;` `\!` `\:` and any other `\`+punctuation sequence inside math.
`\`+letter sequences (e.g. `\qquad`, `\bigl`) are safe.

`\operatorname` is on GitHub's macro denylist. Use `\mathop{\text{...}}` instead — it gives
correct operator spacing. Plain `\text{}` works but lacks operator spacing.

Use `^\circ` for degrees inside math expressions, not the Unicode `°` character.

### Underscores inside `\text{}`

GitHub's markdown preprocessor converts `\_` to `_` before MathJax sees it, so literal underscores
inside `\text{}` break rendering. Use the **backtick-dollar** delimiter to protect the expression
from markdown preprocessing:

- Inline: `` $`\text{accel\_end}`$ `` (backtick after opening `$`, backtick before closing `$`)
- Display block: use a `` ```math `` fenced code block instead of `$$...$$`

The backtick tells the markdown parser to treat the contents as code, passing them through to
MathJax untouched. Only needed when the expression contains characters that conflict with markdown
(underscores, asterisks, etc.). For expressions without such conflicts, plain `$...$` and
`$$...$$` are preferred.
