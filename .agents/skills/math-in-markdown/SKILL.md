---
name: math-in-markdown
description: "ONLY for .md files. Apply correct math formatting conventions when writing or editing a markdown file that contains formulas or equations. Do NOT use for code comments, Java, or any non-markdown file."
---

Apply the following conventions when writing math in `.md` files.

## Core rule

**Protect non-trivial math, then write normal LaTeX.**

Do not rewrite good notation into `\text{...}` wrappers as a GitHub workaround. Use `\text{...}` or `\mathrm{...}` only when you want upright labels (words, multi-letter names), not to dodge the markdown preprocessor.

## Delimiters

### Inline

- Simple symbols with no markdown-sensitive characters: `$x$`, `$k_P$`, `$\alpha$`
- Anything with accents, hats, dots, thin spaces, or other fragile TeX: **backtick-dollar**

  Example (inline, protected):

  - `$` + backtick + `\ddot{x}` + backtick + `$` → renders $\ddot{x}$
  - Same pattern for `\hat{f}`, `(x, \dot{x}, f)`, `\omega_o`

  The backticks wrap the TeX so markdown leaves it alone for MathJax.

### Display (body content, outside `<details>`)

Prefer a fenced math block so you get both rendering and the gray code-block chrome.

Use a language-`math` fence:

- Opening line: triple-backtick + `math`
- Body: normal LaTeX (e.g. `\ddot{x} = b \cdot u + f`)
- Closing line: triple-backtick

Plain `$$...$$` also renders, but has no gray box.

### Display (inside `<details>` / `<summary>`)

**Fenced language-`math` blocks do not reliably render as math inside HTML `<details>` on GitHub.** They often stay as raw LaTeX in a code fence.

Use dollar display math instead:

```markdown
$$\ddot{x} = b \cdot u + f$$
```

Optional visual frame (left bar / tinted region; not the same as the code-block gray box):

```markdown
> $$\ddot{x} = b \cdot u + f$$
```

Keep inline math in details as plain `$...$` or backtick-dollar as above; those work inside details.

## Desmos compatibility

Use Desmos function names wherever a Desmos equivalent exists (e.g. `sign` not `sgn`). Desmos
uses `arctan(y, x)` as its two-argument atan2.

## GitHub rendering pitfalls

GitHub's markdown preprocessor runs **before** MathJax. It can:

- Strip backslash + punctuation sequences (`\,` `\;` `\!` `\:` `\_`, etc.). Backslash + letter sequences (`\qquad`, `\ddot`, `\hat`, `\bigl`) are valid TeX, but the whole expression still needs protection if surrounding markdown would mangle it.
- Turn bare `_` into emphasis mid-expression.
- Treat content inside HTML blocks (especially `<details>`) differently from top-level markdown.

**Mitigation:**

- Fragile inline → backtick-dollar
- Display in normal body → language-`math` fence (gray box + render)
- Display inside `<details>` → `$$...$$` or blockquote-framed `> $$...$$`

`\operatorname` is on GitHub's macro denylist. Use `\mathop{\text{...}}` instead — it gives
correct operator spacing. Plain `\text{}` works but lacks operator spacing.

Use `^\circ` for degrees inside math expressions, not the Unicode `°` character.

## Quick chooser

| Context | Use |
| -------- | --- |
| Inline, simple | `$x$`, `$k_V$` |
| Inline, accents / complex | backtick-dollar around the TeX |
| Display, top-level body | language-`math` fence |
| Display, inside `<details>` | `$$...$$` or `> $$...$$` |
| Upright multi-letter label | `\text{target}` or `\mathrm{target}` (not as an escape hatch) |

## Examples

**Body (outside details):** language-`math` fence for display; backtick-dollar for hats/dots inline.

**Inside a collapsible aside:**

```markdown
<details>
<summary><strong>Advanced aside</strong></summary>

Once you have the estimate $`\hat{f}`$, cancel it:

> $$u = \frac{u_0 - \hat{f}}{b_0}$$

</details>
```
