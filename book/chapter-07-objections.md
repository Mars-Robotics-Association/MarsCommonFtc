# Chapter 7 Objections

## 7.2 -- RMS jerk claim is backwards

> "the *RMS* jerk is lower because the sinusoidal shape distributes the impulse more evenly"

Both halves of this sentence are wrong.

The RMS jerk of the sinusoidal transition is *higher*, not lower. The
sinusoidal jerk is $j(t') = \frac{\pi j_{\max}}{2}\sin(\pi t'/T)$. Its RMS
over one transition is:

$$j_{\text{RMS,sin}} = \frac{\pi j_{\max}}{2}\cdot\frac{1}{\sqrt{2}}
  = \frac{\pi j_{\max}}{2\sqrt{2}} \approx 1.11\,j_{\max}$$

The linear S-curve has constant jerk $j_{\max}$ throughout, so its RMS is
exactly $j_{\max}$. The sinusoidal RMS is about 11% higher.

The claim that the sinusoidal shape "distributes the impulse more evenly" is
also backwards. Constant jerk is the most evenly distributed jerk possible --
it is literally uniform. The sinusoidal shape concentrates jerk in the middle
of the transition and tapers to zero at the edges. That tapering is exactly
the *point* (it buys boundary continuity), but it is the opposite of "more
evenly distributed."

Suggested fix: drop the RMS claim entirely. The real win is that jerk is
continuous at the phase boundaries, not that any bulk jerk metric improves.

## 7.3 -- "the $\frac{1}{6}$ factor" implies a single constant in the linear S-curve

> "These replace the $\frac{1}{6}$ factor from the cubic polynomial distance
> formula of the linear S-curve."

The linear S-curve has two different distance factors: $\frac{1}{6}$ for
onset and $\frac{1}{3}$ for offset. Saying "the $\frac{1}{6}$ factor" (singular)
suggests both sinusoidal constants replace a single value, which is misleading.

Suggested fix: say something like "These replace the $\frac{1}{6}$ and
$\frac{1}{3}$ factors from the onset and offset distance formulas of the
linear S-curve."

## 7.5 -- "connects smoothly with whatever came before" over-promises

> "The jerk at $t' = 0$ is zero (cosine derivative is $-\sin$, which is zero
> at the origin), so the prefix connects smoothly with whatever came before."

The prefix's jerk is zero at its start, but the jerk of the preceding
trajectory segment is generally non-zero. So there is still a jerk
discontinuity at the splice point. The text makes it sound like jerk
continuity is guaranteed with the prior motion, which section 7.10 later
correctly disclaims for replans.

Suggested fix: say the prefix connects with *continuous acceleration* (which
is true and is what the a0 prefix is designed to ensure). Reserve the word
"smoothly" for cases where jerk is also matched, or qualify it: "so
acceleration is continuous at the splice, though jerk is not."
