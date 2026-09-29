#!/usr/bin/env python3
"""Minimal Markdown -> LaTeX -> PDF converter.

Tailored to the constructs used by docs/labview_interface.md (headings,
pipe tables, fenced code blocks, blockquotes, bullet/numbered lists with
one level of nesting, bold/italic/inline-code). Not a general Markdown
engine -- it exists so this project's hand-written reference docs can be
rendered to PDF with the system pdflatex (no pandoc dependency).

Usage:
    python3 python/md_to_pdf.py docs/labview_interface.md docs/labview_interface.pdf
"""
import os
import re
import sys
import subprocess
import tempfile

TEXLIVE_BIN = "/usr/local/texlive/2023/bin/universal-darwin"


# --------------------------------------------------------------------------
# Inline text conversion
# --------------------------------------------------------------------------

def esc_text(s):
    """Escape LaTeX specials + map the unicode chars this project uses.

    Uses placeholder tokens so that LaTeX introduced by the unicode
    mapping is NOT re-escaped by the special-char pass."""
    s = s.replace("\\", "@@BS@@")
    s = s.replace("\u2014", "@@MDASH@@").replace("\u2013", "@@NDASH@@")
    s = s.replace("\u2192", "@@RARR@@").replace("\u2264", "@@LEQ@@")
    s = s.replace("\u00a7", "@@SECT@@")
    s = (s.replace("&", r"\&").replace("%", r"\%").replace("$", r"\$")
          .replace("#", r"\#").replace("_", r"\_")
          .replace("{", r"\{").replace("}", r"\}")
          .replace("~", r"\textasciitilde{}").replace("^", r"\textasciicircum{}"))
    s = s.replace("@@BS@@", r"\textbackslash{}")
    s = s.replace("@@MDASH@@", "---").replace("@@NDASH@@", "--")
    s = s.replace("@@RARR@@", r"$\rightarrow$").replace("@@LEQ@@", r"$\leq$")
    s = s.replace("@@SECT@@", r"\S{}")
    return s


def esc_code(s):
    """Escape content destined for \\texttt{...}."""
    s = s.replace("\\", "@@BS@@")
    s = (s.replace("{", r"\{").replace("}", r"\}")
          .replace("_", r"\_").replace("#", r"\#").replace("%", r"\%")
          .replace("$", r"\$").replace("&", r"\&")
          .replace("~", r"\textasciitilde{}").replace("^", r"\textasciicircum{}"))
    s = s.replace("@@BS@@", r"\textbackslash{}")
    return s


def inline(s, _depth=0):
    """Convert inline Markdown to LaTeX.

    Recursive tokenizer for `code`, **bold**, and *italic*, so they may
    nest (e.g. **Use `SHOT:STARt` (not `SOURce:RUN`)**)."""
    if _depth > 12:
        return esc_text(s)
    s = s.replace(r"\|", "|")  # my table cells use \| to escape pipes
    out = []
    i, n = 0, len(s)
    while i < n:
        c = s[i]
        if c == "`":
            j = s.find("`", i + 1)
            if j == -1:
                out.append(esc_text(s[i:])); break
            out.append(r"\texttt{" + esc_code(s[i + 1:j]) + "}")
            i = j + 1
        elif s.startswith("**", i):
            j = s.find("**", i + 2)
            if j == -1:
                out.append(esc_text(s[i:])); break
            out.append(r"\textbf{" + inline(s[i + 2:j], _depth + 1) + "}")
            i = j + 2
        elif c == "*":
            j = s.find("*", i + 1)
            if j == -1:
                out.append(esc_text(s[i:])); break
            out.append(r"\emph{" + inline(s[i + 1:j], _depth + 1) + "}")
            i = j + 1
        else:
            j = i
            while j < n and s[j] not in "`*":
                j += 1
            out.append(esc_text(s[i:j]))
            i = j
    return "".join(out)


# --------------------------------------------------------------------------
# Table conversion
# --------------------------------------------------------------------------

def split_row(row):
    cells, cur, i = [], "", 0
    while i < len(row):
        c = row[i]
        if c == "\\" and i + 1 < len(row) and row[i + 1] == "|":
            cur += "|"
            i += 2
            continue
        if c == "|":
            cells.append(cur)
            cur = ""
            i += 1
            continue
        cur += c
        i += 1
    cells.append(cur)
    if cells and cells[0].strip() == "":
        cells = cells[1:]
    if cells and cells[-1].strip() == "":
        cells = cells[:-1]
    return cells


def emit_table(rows, out):
    header = split_row(rows[0])
    ncol = len(header)
    spec = {2: "p{3.2cm}p{12.4cm}",
            3: "p{4.8cm}p{4.4cm}p{6.3cm}"}.get(ncol, "l" * ncol)
    out.append("\\begin{center}\\small")
    out.append("\\begin{tabular}{%s}" % spec)
    out.append("\\hline")
    out.append(" & ".join(inline(c.strip()) for c in header) + " \\\\")
    out.append("\\hline")
    for r in rows[2:]:
        cells = split_row(r)
        while len(cells) < ncol:
            cells.append("")
        out.append(" & ".join(inline(c.strip()) for c in cells[:ncol]) + " \\\\")
    out.append("\\hline")
    out.append("\\end{tabular}")
    out.append("\\end{center}")


# --------------------------------------------------------------------------
# Block conversion
# --------------------------------------------------------------------------

LIST_BULLET = re.compile(r"^(\s*)[-*]\s+(.*)$")
LIST_NUM = re.compile(r"^(\s*)(\d+)\.\s+(.*)$")


def convert(md_text):
    lines = md_text.split("\n")
    out = []
    title = None
    list_stack = []  # (indent, env)
    i, n = 0, len(lines)

    def close_lists():
        while list_stack:
            _, env = list_stack.pop()
            out.append("\\end{%s}" % env)

    while i < n:
        line = lines[i]
        st = line.strip()

        # fenced code block
        if st.startswith("```"):
            buf = []
            i += 1
            while i < n and not lines[i].strip().startswith("```"):
                buf.append(lines[i])
                i += 1
            i += 1  # closing fence
            close_lists()
            out.append("\\begin{verbatim}")
            out.extend(buf)
            out.append("\\end{verbatim}")
            continue

        if st == "":
            close_lists()
            out.append("")
            i += 1
            continue

        # horizontal rule (section separator) -- skip
        if st == "---" or set(st) <= set("- "):
            i += 1
            continue

        # headings
        if st.startswith("#"):
            close_lists()
            level = len(st) - len(st.lstrip("#"))
            heading = inline(st[level:].strip())
            if level == 1:
                title = heading
            elif level == 2:
                out.append("\\section{%s}" % heading)
            else:
                out.append("\\subsection{%s}" % heading)
            i += 1
            continue

        # table
        if st.startswith("|"):
            close_lists()
            rows = []
            while i < n and lines[i].strip().startswith("|"):
                rows.append(lines[i])
                i += 1
            emit_table(rows, out)
            continue

        # blockquote
        if st.startswith(">"):
            close_lists()
            qbuf = []
            while i < n and lines[i].strip().startswith(">"):
                qbuf.append(lines[i].strip()[1:].strip())
                i += 1
            out.append("\\begin{quote}")
            out.append(inline(" ".join(qbuf)))
            out.append("\\end{quote}")
            continue

        # list items (bullet or numbered)
        mb = LIST_BULLET.match(line)
        mn = LIST_NUM.match(line)
        if mb or mn:
            if mb:
                indent, kind, text = len(mb.group(1)), "itemize", mb.group(2)
            else:
                indent, kind, text = len(mn.group(1)), "enumerate", mn.group(2)
            # gather continuation lines (indented, no new marker)
            j = i + 1
            cont = []
            while j < n and lines[j].strip() != "" \
                    and not LIST_BULLET.match(lines[j]) \
                    and not LIST_NUM.match(lines[j]) \
                    and not lines[j].strip().startswith(("#", "|", ">", "```")) \
                    and not (lines[j].strip() == "---"
                             or set(lines[j].strip()) <= set("- ")):
                cont.append(lines[j].strip())
                j += 1
            if cont:
                text = text + " " + " ".join(cont)
                i = j
            else:
                i += 1
            while list_stack and indent < list_stack[-1][0]:
                _, env = list_stack.pop()
                out.append("\\end{%s}" % env)
            if not list_stack or indent > list_stack[-1][0]:
                out.append("\\begin{%s}" % kind)
                list_stack.append((indent, kind))
            out.append("\\item " + inline(text))
            continue

        # paragraph
        close_lists()
        para = [st]
        j = i + 1
        while j < n and lines[j].strip() != "" \
                and not lines[j].strip().startswith(("#", "|", ">", "```")) \
                and not LIST_BULLET.match(lines[j]) \
                and not LIST_NUM.match(lines[j]) \
                and not (lines[j].strip() == "---"
                         or set(lines[j].strip()) <= set("- ")):
            para.append(lines[j].strip())
            j += 1
        i = j
        out.append(inline(" ".join(para)))
        out.append("")

    close_lists()
    return title, out


# --------------------------------------------------------------------------
# Driver
# --------------------------------------------------------------------------

PREAMBLE = r"""\documentclass[11pt]{article}
\usepackage[utf8]{inputenc}
\usepackage[T1]{fontenc}
\usepackage{geometry}
\geometry{margin=0.9in}
\usepackage{array}
\setlength{\parindent}{0pt}
\setlength{\parskip}{5pt}
\emergencystretch=2em
\title{%s}
\date{}
\begin{document}
\maketitle
"""


def main():
    if len(sys.argv) < 3:
        sys.exit("usage: md_to_pdf.py <in.md> <out.pdf>")
    src, dst = sys.argv[1], sys.argv[2]
    md = open(src, encoding="utf-8").read()
    title, body = convert(md)
    tex = PREAMBLE % (title or "Document")
    tex += "\n".join(body)
    tex += "\n\\end{document}\n"

    with tempfile.TemporaryDirectory() as td:
        texpath = os.path.join(td, "doc.tex")
        with open(texpath, "w", encoding="utf-8") as f:
            f.write(tex)
        env = dict(os.environ)
        env["PATH"] = TEXLIVE_BIN + os.pathsep + env.get("PATH", "")
        r = subprocess.run(
            ["pdflatex", "-interaction=nonstopmode", "-halt-on-error",
             "doc.tex"],
            cwd=td, env=env, capture_output=True, text=True)
        if r.returncode != 0:
            sys.stderr.write(r.stdout)
            sys.stderr.write(r.stderr)
            sys.exit("pdflatex failed (exit %d)" % r.returncode)
        pdf = os.path.join(td, "doc.pdf")
        if not os.path.exists(pdf):
            sys.exit("pdflatex produced no PDF")
        with open(pdf, "rb") as fsrc, open(dst, "wb") as fdst:
            fdst.write(fsrc.read())
    print("wrote %s" % dst)


if __name__ == "__main__":
    main()
