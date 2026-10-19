# Frontier exploration: research drafts

Two manuscripts describe the frontier explorer on `main`. They were written from the source and the commit history, not from the June 2026 note. That note disagrees with the current code and is treated as a historical document.

| Document | What it is |
|---|---|
| [iteration1_concise/main.pdf](iteration1_concise/main.pdf) | Conference-style argument (13 pages). Method, score, failure policy, evidence gap. |
| [iteration2_comprehensive/main.pdf](iteration2_comprehensive/main.pdf) | Technical specification (about 30 pages). Parameters, pseudocode, history, legacy-note audit, protocol. |
| [assessment/publication_ip_assessment.pdf](assessment/publication_ip_assessment.pdf) | Whether this is ready for IROS/ICRA/RA-L, and whether a patent is a plausible next step. Not legal advice. |
| [SOURCE_TRACEABILITY.md](SOURCE_TRACEABILITY.md) | Claim to file, line, and commit. |

Figures live in `figures/all_figures.tex` and are shared. They are diagrams of the software, not pictures of the robot. Quantitative charts of exploration performance are absent because the repository has no such measurements. Worked score diagrams are labelled as arithmetic examples.

Rebuild with [Tectonic](https://tectonic-typesetting.github.io/) from each directory:

```bash
tectonic main.tex
```

Do not cite these drafts as experimental results. The explorer does not, on the present evidence, have a measured advantage over nearest-frontier or `explore_lite`.
