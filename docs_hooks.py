"""MkDocs hook for the documentation website: links from docs/ to files outside it (the firmware,
the Pi, hardware, code) point to those files on GitHub, so they work on the website as they do
when reading the Markdown on GitHub."""
from __future__ import annotations

import re
from pathlib import Path

REPO = "https://github.com/jolianjij/robofield"
LINK = re.compile(r"\]\((?!https?://|mailto:|#)([^)\s]+)\)")


def on_page_markdown(markdown, page, config, files):
    docs = Path(config["docs_dir"]).resolve()
    root = docs.parent
    here = (docs / page.file.src_path).parent

    def fix(m):
        target = m.group(1)
        path, _, anchor = target.partition("#")
        if not path:
            return m.group(0)
        dest = (here / path).resolve()
        try:
            dest.relative_to(docs)
            return m.group(0)                      # inside docs/: MkDocs handles it
        except ValueError:
            pass
        try:
            rel = dest.relative_to(root).as_posix()
        except ValueError:
            return m.group(0)
        kind = "tree" if dest.is_dir() else "blob"
        return f"]({REPO}/{kind}/main/{rel}{'#' + anchor if anchor else ''})"

    # leave code blocks alone
    parts = re.split(r"(```.*?```)", markdown, flags=re.S)
    return "".join(p if p.startswith("```") else LINK.sub(fix, p) for p in parts)
