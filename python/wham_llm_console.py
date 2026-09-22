#!/usr/bin/env python3
"""
wham_llm_console.py -- LLM-fronted operator console for WHAM-XREX-PFMG474.

An ALTERNATE FRONT END to wham_console.py with identical capabilities: the
operator talks in plain English to a small, open-weight LLM running LOCALLY
on this machine, and the LLM decides which console/SCPI commands to send,
then reports the results back in plain English. Underneath, this script
drives a real wham_console.WhamConsole instance (imported, unmodified), so
every console command, the interactive `shot` wizard, automatic plotting,
`reflash` integration, session logging, and raw SCPI passthrough behave
EXACTLY as they do in wham_console.py -- this file adds no new wire
behavior of its own.

How it works
------------
1. The operator types natural language ("check the board is healthy",
   "set up a 3 kA, 5 s ramp / 2 s flat-top shot on channel 1, closed loop,
   then fire it and show me the plot").
2. The message, a system prompt, and the recent conversation are sent to a
   local LLM over an OpenAI-compatible HTTP API. The system prompt teaches
   the model (a) what this project IS (the Transrex/WHAM closed-loop
   controller -- condensed from AGENTS.md, Core/Inc/ctrlr_config.h, pid.h,
   and state_machine.h, including which calibration values are unverified
   placeholders and the current bench reality), (b) its ROLE (the operator's
   hands and interpreter; the human stays in charge), (c) the operator's
   GOALS (bring-up -> characterization -> gain tuning -> production shots),
   and (d) the full console + SCPI command catalog (condensed from
   docs/command_reference.md).
3. The model replies with ONE JSON object:
       {"say": "<plain text shown to the operator>",
        "actions": ["<console command line>", ...]}
   Each action is a command line EXACTLY as a human would type it into
   wham_console.py ("status", "gains 1 1.0 10.0 0.0", "FAULT?",
   "SHOT:TIMing 5 2 5", "plot all", ...). Actions run in order and
   their captured output is sent back to the model, which may then act
   again (multi-step turns, e.g. program -> ARM -> START -> wait -> plot)
   or finish with empty actions.
4. SAFETY GATE: any action that can start real PWM output or reflash
   firmware (FIRE, BOOT, SOURce:RUN, SHOT:STARt in any SCPI short/long
   form, plus the `start`, `shot`, `reflash`, and `enable <ch> on` wrappers)
   is confirmed with the OPERATOR in the terminal before it runs -- the LLM
   cannot bypass this, it can only propose. `--auto` / `/auto on` disables
   the gate for trusted hands-off use. This mirrors (and reuses)
   wham_console.py's own confirm machinery -- see _is_dangerous() there.

LLM backend -- OpenAI-compatible local endpoint (Mac + Windows)
---------------------------------------------------------------
Any server that speaks POST /v1/chat/completions works. Easiest options on
consumer hardware, both free and cross-platform (macOS and Windows):

  * Ollama (recommended default):  https://ollama.com
        ollama pull qwen3:1.7b      # one-time model download (~1.4 GB)
    Ollama serves http://localhost:11434/v1 automatically once installed
    and running. This script's defaults point straight at it.
    NOTE -- Ollama's default context window is only 4096 tokens, and this
    script's system prompt (project briefing + command catalog, including
    the `diag`/`report` debugging tools) is ~3700-4700 of that BY ITSELF
    (grows with channel count and conversation), before any history or the
    model's own reply -- 4096 is too tight for real use. Raise the window
    once when starting the server:
        OLLAMA_CONTEXT_LENGTH=16384 ollama serve
    (LM Studio equivalent: the per-model "Context Length" slider.) Also
    raise the keep-alive so the model doesn't reload between uses:
        OLLAMA_KEEP_ALIVE=30m ollama serve
    (macOS running Ollama.app rather than the CLI: `launchctl setenv
    OLLAMA_CONTEXT_LENGTH 16384` + `launchctl setenv OLLAMA_KEEP_ALIVE
    30m`, then quit and relaunch the app -- session-scoped, redo after a
    reboot. Neither takes effect if passed per-request through this
    endpoint instead -- confirmed via `ollama ps` on 2026-09-14, both a
    context-length "options" field and a "keep_alive" field are silently
    ignored here; they have to be set server-side.)
  * LM Studio:  https://lmstudio.ai  -- download a model in the GUI, then
    "Developer" -> "Start Server" (default http://localhost:1234/v1) and
    pass --endpoint http://localhost:1234/v1 --model <loaded model id>.

Default model: qwen3:1.7b (Apache-2.0; really a 2.0B-parameter checkpoint
per `ollama show` -- Ollama's tag name undersells it slightly). Chosen
over the earlier default qwen3:4b after a real head-to-head benchmark,
2026-09-14, same real system prompt and real requests against real
hardware: 4-9x faster (9.6s vs 41.6s on a simple health check; 4.8s vs
never-finished on a "generate a report" request -- 1.7b actually called
the `report` tool where 4b's own two real attempts never did), smaller
memory footprint (3.3GB vs 5.1GB loaded). CAVEAT, not a speed tradeoff --
a behavioral one: on a complex multi-parameter shot-setup request, 1.7b's
own reply text said "I'll start output without your go-ahead," a
noticeably more cavalier tone than 4b's equivalent replies (which said
"Confirm to proceed?"). This is NOT an actual safety gap -- the real gate
(Executor.is_dangerous()) is enforced in code, completely independent of
what the model's own text claims, and it still asked for real y/N
confirmation before SHOT:STARt regardless -- but it means reading
every confirm prompt on its own merits matters even more with this
model, not less. Switch back with `--model qwen3:4b` (or `/model
qwen3:4b` live) if that tradeoff doesn't sit right -- both are
documented, tested choices, not "one replaced the other." Other good
small open-weight choices: llama3.2:3b, phi4-mini, qwen2.5:3b-instruct.
Bigger is better for multi-step shot programming; anything under ~1.5B
will likely struggle with the JSON output contract. Model replies have
any <think>...</think> blocks stripped before parsing, so reasoning-
style models (qwen3 et al.) work as-is.

QUANTIZATION: both qwen3:1.7b and qwen3:4b are ALREADY Q4_K_M (4-bit) --
`ollama pull` uses this by default, not full precision, so "should we
quantize" is already answered yes. Tried pulling more aggressive tags
(qwen3:4b-q4_0, qwen3:4b-q3_K_M) directly from Ollama's library,
2026-09-14 -- neither exists; Ollama only publishes this one quant for
this model. A lower quant would need hand-importing a GGUF from Hugging
Face, not attempted -- diminishing returns expected (dropping bit-depth
on the SAME parameter count saves much less than dropping parameter
count did, with more quality loss per bit at this point) against the
much bigger, already-validated win of the smaller model above.

SPEED -- qwen3 "thinks" before every reply, and it's expensive: measured
on a real M2 Pro/16GB against real hardware, 2026-09-14, a trivial two-
action health-check decision took 112s wall time on qwen3:4b, ALL of it
generating ~2800 tokens of chain-of-thought for a decision that needed
none. This script appends Qwen3's own "/no_think" convention to every
turn by default (LLMClient.suppress_thinking, NO_THINK_SUFFIX) -- the
SAME request measured 30s with it on (qwen3:4b) / 9.6s (qwen3:1.7b),
same correct actions, reasoning REDUCED not eliminated either way.
Toggle live with `/think on` if tool-selection seems to suffer without
deep reasoning (not fully characterized either way). Ollama's own native
"think": false parameter (the "proper" toggle) does NOT work on a
default Ollama+qwen3 setup that serves with a generic chat template
(confirmed on this exact setup) -- the plain-text convention is the
actual fix, not a fallback.

Prerequisites:
  pip install pyserial          (required -- same as wham_console.py)
  pip install matplotlib        (optional -- only for `plot`/`shot` plots)
  a running local LLM server as above (required for natural-language turns;
                                  /raw and all /-commands work without one)

Usage:
  python3 python/wham_llm_console.py                    # auto-detect port + LLM
  python3 python/wham_llm_console.py --port /dev/cu.usbserial-130
  python3 python/wham_llm_console.py --port COM3                 # Windows
  python3 python/wham_llm_console.py --model llama3.2:3b
  python3 python/wham_llm_console.py --endpoint http://localhost:1234/v1
  python3 python/wham_llm_console.py --no-connect                # start disconnected
  python3 python/wham_llm_console.py --auto   # skip per-command confirm (TRUSTED use)

Environment variables: WHAM_LLM_ENDPOINT, WHAM_LLM_MODEL override the
endpoint/model defaults (command-line flags win over both).

Once running:
  you> anything typed here goes to the LLM
  /raw <line>   run one wham_console command line directly, bypassing the LLM
                (same confirm gate; e.g. `/raw *IDN?`, `/raw status`)
  /auto [on|off]    toggle the dangerous-command confirm gate (default off)
  /think [on|off]   toggle Qwen3's "/no_think" speed trick (default ON --
                     ~3.7x faster measured on real hardware, see NO_THINK_SUFFIX)
  /model [name]     show or switch the current endpoint/model; /models lists server models
  /clear            forget the conversation so far (device state is unaffected)
  /help             show the local command list again
  /quit             close the serial link and exit
Anything the LLM does is printed verbatim ($ command + device reply) so the
operator always sees exactly what went over the wire; the session wire log
in logs/ is the same one wham_console.py writes, with LLM turns added as
"#"-prefixed lines.
"""

import argparse
import contextlib
import glob
import io
import json
import os
import re
import sys
import time
import urllib.error
import urllib.request
from collections import namedtuple
from datetime import datetime

# This script lives in python/ alongside wham_console.py; make that import
# work no matter what directory it was launched FROM (sys.path[0] is already
# the script's own dir in the normal `python3 python/wham_llm_console.py`
# case -- this is purely defensive).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import wham_console as wc  # the maintained human console, reused unmodified

# --------------------------------------------------------------------------
# Defaults / constants
# --------------------------------------------------------------------------

DEFAULT_ENDPOINT = os.environ.get("WHAM_LLM_ENDPOINT", "http://localhost:11434/v1")
DEFAULT_MODEL = os.environ.get("WHAM_LLM_MODEL", "qwen3:1.7b")

LLM_TIMEOUT_S = 300.0   # small models on CPU can be slow to first token
LLM_TEMPERATURE = 0.2   # low -- we want obedient JSON, not creativity
LLM_MAX_TOKENS = 6000   # reasoning models (qwen3 et al., the recommended default)
                        # spend an unpredictable chunk of this on their own
                        # reasoning before the actual JSON answer -- too tight
                        # a cap here risks the reply getting cut off mid-
                        # thought (strip_think() handles an INLINE unclosed
                        # <think> defensively, see its docstring, but a
                        # bigger budget means it happens less often in the
                        # first place). Raised 3000->6000, 2026-09-14, with
                        # real evidence, not a guess: a real multi-parameter
                        # request's session log (log_llm_reply(), see
                        # docs/changelog.txt) showed 3 of its first 6 rounds
                        # hit EXACTLY 3000 completion tokens and produced NO
                        # valid JSON at all -- the model was still reasoning
                        # when the cap cut it off, wasting the entire round
                        # (60-82s each) on a forced retry. Every round that
                        # DID succeed used well under 3000 (1146-2416) --
                        # consistent with "about to finish, cut off early,"
                        # not "rambling forever" -- so a higher ceiling should
                        # let more rounds finish on the first try instead of
                        # needing a wasted retry, not just let it ramble
                        # longer. Not fully re-characterized across many
                        # requests -- if a session log shows repeated exactly-
                        # 6000-token rounds, this may need raising further.

NO_THINK_SUFFIX = " /no_think"
# Qwen3's own documented soft-switch convention (append to the latest turn
# to request a non-reasoning reply) -- NOT a formal API parameter, just text
# the model was trained to respect. Tested against Ollama's *native*
# /api/chat "think": false parameter first (the "proper" toggle) -- it did
# NOT work on this setup: this Ollama build serves qwen3 with
# --chat-template chatml --no-jinja (a generic template, not qwen3's own
# jinja template that "think" actually hooks into), so reasoning showed up
# INLINE in "content" regardless of the "think" flag, breaking the JSON
# contract. The plain-text /no_think suffix works BECAUSE it doesn't depend
# on that template plumbing -- confirmed via a real measured 112s->30s
# round trip on this exact box. Only meaningful for Qwen3 models; harmless
# (just unused trailing text) if pointed at a model that doesn't recognize
# it, so no model-name gating -- see LLMClient.chat().

MAX_ROUNDS_PER_TURN = 6   # LLM acts -> sees results -> acts again, capped
MAX_ACTIONS_PER_ROUND = 12
HISTORY_KEEP = 20         # messages kept (small models have small contexts --
                          # system prompt is ~2.5k tokens; Ollama's default
                          # context window is only 4096, see header docstring)
WAIT_CAP_S = 60.0         # ceiling on the LLM's `wait <seconds>` pseudo-action
MAX_RESULT_CHARS = 1500   # cap on one command's captured output before it goes
                          # into LLM history -- raw `LOG:DATA? <ch>` (up to
                          # 1000 samples on one line) can otherwise blow straight
                          # through a small local model's whole context window in
                          # a single command result; `plot`/`log` print file paths,
                          # not data, and are unaffected. The OPERATOR still sees
                          # the untruncated reply in the terminal -- only what goes
                          # back to the model is capped.

# Console commands whose whole point is an interactive terminal session with
# the OPERATOR (input() prompts / streaming until Ctrl-C). These are run
# live, NOT under redirected stdout, and the LLM only gets back a note --
# see Executor.run().
INTERACTIVE_COMMANDS = {"shot", "monitor", "reflash"}

# Console wrapper commands the LLM must never run (the operator exits
# themselves with /quit). Filtered before execution.
FORBIDDEN_COMMANDS = {"quit", "exit", "eof"}


def find_port_any():
    """wham_console.find_port() only globs /dev/* (macOS/Linux). For Windows
    parity, fall back to pyserial's own port enumeration and take the first
    port that looks like a USB-serial device."""
    port = wc.find_port()
    if port:
        return port
    try:
        from serial.tools import list_ports
        for p in list_ports.comports():
            text = f"{p.device} {p.description or ''} {p.hwid or ''}"
            if re.search(r"usb|serial|cp210|ch340|ftdi|uart", text, re.IGNORECASE):
                return p.device
    except Exception:
        pass
    return None


# --------------------------------------------------------------------------
# LLM client -- minimal stdlib OpenAI-compatible /chat/completions caller.
# --------------------------------------------------------------------------

class LLMError(Exception):
    """Transport- or server-level failure talking to the LLM endpoint (not
    reachable, HTTP error, malformed response). Reported to the operator;
    the console session itself keeps running."""


LLMReply = namedtuple("LLMReply", ["text", "reasoning", "elapsed_s", "usage"])
# text: the JSON-contract answer (already <think>-stripped). reasoning:
# separate chain-of-thought text if the backend/model returned one (Ollama-
# specific "reasoning" field -- see LLMClient.chat()'s own comment; empty
# string if none). elapsed_s: wall time for this ONE HTTP call, for the
# session log (see log_llm_reply()) -- not the whole turn, which may be
# several rounds. usage: the raw "usage" dict from the API reply (prompt/
# completion token counts), or {} if the backend didn't send one.


class LLMClient:
    def __init__(self, endpoint, model, timeout=LLM_TIMEOUT_S, suppress_thinking=True):
        self.base = endpoint.rstrip("/")
        self.model = model
        self.timeout = timeout
        # See NO_THINK_SUFFIX's own comment -- measured on real hardware,
        # 2026-09-14: qwen3:4b spent 2815 of 2815 completion tokens (112s
        # wall time) THINKING about a trivial two-action health-check
        # decision. Appending "/no_think" cut that to 30s (~3.7x) with the
        # same correct actions. Default ON since speed was the direct
        # request that led to this; toggle live with /think on|off if
        # tool-selection reliability seems to suffer without deep
        # reasoning (not fully characterized either way -- see
        # docs/changelog.txt's 2026-09-14 entry for what WAS tested).
        self.suppress_thinking = suppress_thinking

    def _post(self, path, payload):
        req = urllib.request.Request(
            self.base + path,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            try:
                body = exc.read().decode("utf-8", errors="replace")[:400]
            except Exception:
                body = ""
            raise LLMError(f"LLM server returned HTTP {exc.code}: {body}") from exc
        except urllib.error.URLError as exc:
            raise LLMError(
                f"couldn't reach the LLM server at {self.base} ({exc.reason}).\n"
                "Is one running? Easiest: install Ollama (https://ollama.com), then\n"
                f"  ollama pull {self.model}\n"
                "and re-run this script (Ollama serves http://localhost:11434/v1 on its own).\n"
                "For LM Studio, start its local server and pass "
                "--endpoint http://localhost:1234/v1 --model <id>."
            ) from exc

    def chat(self, messages):
        """One non-streaming chat completion. Returns the assistant message's
        content string with any <think>...</think> blocks stripped (qwen3 et
        al. -- the JSON contract applies to what remains).

        When self.suppress_thinking, appends NO_THINK_SUFFIX to a COPY of
        the last message before sending -- never mutates the caller's own
        `messages`/history, so /no_think never ends up literally stored or
        replayed back to the model as if the operator typed it. Applied to
        whichever message is actually last (the operator's own turn on
        round 1 of a multi-round turn, or a synthesized "Command results:"
        message on a later round) -- Qwen3's own convention is "the latest
        turn," not specifically a human-authored one, so this is applied
        the same way on every round for consistent latency."""
        if self.suppress_thinking and messages and messages[-1].get("role") == "user":
            messages = messages[:-1] + [{**messages[-1],
                                          "content": messages[-1]["content"] + NO_THINK_SUFFIX}]
        t0 = time.time()
        data = self._post("/chat/completions", {
            "model": self.model,
            "messages": messages,
            "temperature": LLM_TEMPERATURE,
            "max_tokens": LLM_MAX_TOKENS,
            "stream": False,
        })
        elapsed_s = time.time() - t0
        try:
            msg = data["choices"][0]["message"]
            content = msg["content"]
        except (KeyError, IndexError, TypeError) as exc:
            raise LLMError(f"unexpected LLM response shape: {str(data)[:300]}") from exc
        # "reasoning" is an Ollama-specific extension of the OpenAI schema
        # (separate <think> content, not inline in "content" on this setup
        # -- see NO_THINK_SUFFIX's docstring); absent/empty on other
        # backends or a non-reasoning model, which is fine, .get() handles it.
        return LLMReply(text=strip_think(content or ""),
                         reasoning=msg.get("reasoning") or "",
                         elapsed_s=elapsed_s,
                         usage=data.get("usage") or {})

    def list_models(self):
        """GET /v1/models -> list of id strings. Raises LLMError if the
        server isn't there -- used at startup to warn early rather than on
        the first natural-language turn."""
        req = urllib.request.Request(self.base + "/models", method="GET")
        try:
            with urllib.request.urlopen(req, timeout=4.0) as resp:
                data = json.loads(resp.read().decode("utf-8"))
        except (urllib.error.URLError, ValueError) as exc:
            raise LLMError(str(exc)) from exc
        return [m.get("id", "?") for m in data.get("data", [])]


def strip_think(text):
    """Strip a model's <think>...</think> reasoning block. If max_tokens cut
    the reply off mid-thought, the closing tag never arrived and the regex
    won't match -- in that case, drop everything from the (still-open)
    <think> onward rather than leaving raw reasoning prose (which may itself
    contain stray '{'/'}' characters) for extract_json() to go hunting
    through. Either way, what's left is either a clean answer or empty --
    never leftover reasoning text."""
    if re.search(r"<think>.*</think>", text, flags=re.DOTALL):
        return re.sub(r"<think>.*?</think>", "", text, flags=re.DOTALL).strip()
    return re.sub(r"<think>.*$", "", text, flags=re.DOTALL).strip()


def truncate_for_llm(text, limit=MAX_RESULT_CHARS):
    """Cap one command result before it enters the LLM's own context -- see
    MAX_RESULT_CHARS. The operator always sees the full, untruncated reply
    in the terminal (Executor.run() prints it before calling this); this
    only shortens what gets fed back to the model."""
    if len(text) <= limit:
        return text
    return (text[:limit] +
            f"\n... (truncated, {len(text) - limit} more chars -- use `plot`/`log` "
            "to see the full data rather than raw LOG:DATA?)")


def extract_json(text):
    """Pull the one JSON object out of a model reply. Tolerates markdown
    code fences and prose around the object (small models add both despite
    instructions); returns None if no parseable object is found."""
    text = text.strip()
    text = re.sub(r"^```(?:json)?\s*", "", text)
    text = re.sub(r"\s*```$", "", text)
    i, j = text.find("{"), text.rfind("}")
    if i < 0 or j <= i:
        return None
    try:
        obj = json.loads(text[i:j + 1])
    except ValueError:
        return None
    return obj if isinstance(obj, dict) else None


def normalize_actions(obj):
    """The contract is actions: ["<command line>", ...], but small models
    sometimes emit {"command": "..."} dicts or a bare string -- accept those
    rather than failing the whole turn."""
    actions = obj.get("actions", [])
    if isinstance(actions, str):
        actions = [actions]
    if not isinstance(actions, list):
        return []
    out = []
    for a in actions:
        if isinstance(a, str) and a.strip():
            out.append(a.strip())
        elif isinstance(a, dict):
            cmd = a.get("command") or a.get("cmd") or a.get("scpi") or ""
            if isinstance(cmd, str) and cmd.strip():
                out.append(cmd.strip())
    return out[:MAX_ACTIONS_PER_ROUND]


# --------------------------------------------------------------------------
# Executor -- runs one LLM-proposed (or /raw) command line against the real
# WhamConsole, applying the human safety gate, and returns the captured
# output text for the LLM.
# --------------------------------------------------------------------------

class Executor:
    def __init__(self, console, auto=False):
        self.console = console
        self.auto = auto  # skip the human gate when True (--auto / `/auto on`)

    @staticmethod
    def _first_token(line):
        return line.strip().split(None, 1)[0] if line.strip() else ""

    def is_dangerous(self, line):
        """Console wrappers that start real output / reflash, PLUS
        wham_console._is_dangerous() for everything else -- including raw
        SCPI. That function is the single shared source of truth (FIRE,
        BOOT, SOURce:RUN, SHOT:STARt, and SOURce:ENAble turning a channel ON) so
        that an LLM proposing raw SCPI instead of a console wrapper (e.g.
        `SOURce:ENAble 1 1` instead of `enable 1 on`) can't slip past
        this gate just by phrasing it differently -- see that function's own
        docstring. Only `start`/`shot`/`reflash` (console-only wrappers with
        no raw-SCPI equivalent) and the `enable` wrapper's English on/off
        spelling need handling here directly; everything else, including
        `enable`'s raw-SCPI form, is covered by passing the FULL line
        through to wc._is_dangerous()."""
        tok = self._first_token(line).lower()
        if tok in ("start", "shot", "reflash"):
            return True
        if tok == "enable":
            parts = line.split()
            return len(parts) >= 3 and parts[2].lower() == "on"
        return wc._is_dangerous(line)

    def _human_allows(self, line):
        try:
            ans = input(f"\n  [!] the LLM wants to run:  {line}\n"
                        f"      This can start real PWM output or reflash firmware. "
                        f"Allow? [y/N] ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            return False
        return ans in ("y", "yes")

    def run(self, line):
        """Execute one command line; return the transcript text that goes
        back to the LLM (device replies / console output / a note explaining
        what happened). Also echoes everything to the operator's terminal so
        nothing the LLM does is hidden."""
        line = line.strip()
        if not line:
            return "(empty action ignored)"

        # `wait <seconds>` -- a LOCAL pseudo-action (never goes to the
        # console), so the LLM can let a shot finish before `plot`. Capped.
        m = re.match(r"^wait\s+(\d+(?:\.\d+)?)\s*$", line, re.IGNORECASE)
        if m:
            s = min(float(m.group(1)), WAIT_CAP_S)
            print(f"  $ wait {s:g}")
            time.sleep(s)
            return f"waited {s:g} seconds"

        tok = self._first_token(line).lower()
        if tok in FORBIDDEN_COMMANDS:
            return ("(quit/exit are not available to you -- the operator exits "
                    "themselves with /quit. Carry on with anything else.)")

        if self.is_dangerous(line) and not self.auto and not self._human_allows(line):
            print("  declined by operator.")
            return ("OPERATOR DECLINED this command -- it was NOT executed. "
                    "Tell the operator, and do not retry it unless they explicitly ask.")

        if tok in INTERACTIVE_COMMANDS:
            # Interactive commands prompt the OPERATOR directly (the `shot`
            # wizard, `monitor`, `reflash`) -- run them with stdout attached
            # to the real terminal, and with the console's own confirm
            # behavior intact. The LLM only gets a note back; the operator
            # experiences exactly wham_console.py's own interactive UX.
            print(f"\n  $ {line}   (interactive -- follow the prompts below)\n")
            try:
                self.console.onecmd(line)
            except Exception as exc:  # a console bug shouldn't kill the session
                print(f"  [console error] {exc}")
                return f"(interactive command raised: {exc})"
            return ("(ran interactively in the terminal; the operator handled its "
                    "prompts and saw its output directly -- it was not captured for you. "
                    "Ask the operator what happened, or query the device yourself, "
                    "e.g. `status` or STATE?/FAULT?)")

        # Non-interactive: capture everything the console would print and
        # hand it to the LLM. If this is a dangerous command the human just
        # approved (or --auto), drop the console's OWN confirm prompt for
        # this one call -- its input() prompt would be invisible inside the
        # redirected stdout and would look like a hang. The gate above is
        # the single point of confirmation, matching wham_console.py's UX
        # (one y/N per dangerous command, not two).
        prev_confirm = self.console.confirm_dangerous
        if self.is_dangerous(line):
            self.console.confirm_dangerous = False
        buf = io.StringIO()
        try:
            with contextlib.redirect_stdout(buf):
                self.console.onecmd(line)
        except Exception as exc:
            buf.write(f"[console error] {exc}\n")
        finally:
            self.console.confirm_dangerous = prev_confirm
        out = buf.getvalue().strip()

        print(f"  $ {line}")
        for ln in out.splitlines():
            print(f"    {ln}")
        return truncate_for_llm(out) or "(no output)"


# --------------------------------------------------------------------------
# System prompt -- the whole contract AND the model's education about this
# project: what WHAM-XREX-PFMG474 is, what its role is, the operator's
# goals, then the command catalog. Condensed from AGENTS.md,
# Core/Inc/ctrlr_config.h, Core/Inc/pid.h, Core/Inc/state_machine.h,
# wham_console.py's command set, and docs/command_reference.md; UPDATE THIS
# when any of those change (same "maintained front end" convention as
# wham_console.py's own header comment).
# --------------------------------------------------------------------------

def build_system_prompt(console):
    n = console.num_channels or 4
    if console.link.connected:
        try:
            idn = console.link.query("*IDN?", timeout=1.0)
        except wc.WhamError:
            idn = "(query failed)"
        conn = f"CONNECTED to {console.link.port} @ {console.link.baud} baud. *IDN? -> {idn}. Channel count (CONFig:CHANnels?) -> {n}."
    else:
        conn = ("NOT currently connected to a board. If the operator asks you to do anything "
                "with the hardware, first run `connect` (auto-detects the USB-serial port) or "
                "`connect <port>` (e.g. /dev/cu.usbserial-130 on a Mac, COM3 on Windows; `ports` lists them).")

    return f"""You are WHAM-LLM, the operator assistant for the WHAM-XREX-PFMG474
controller. The next three sections tell you what this machine IS, what YOUR
role is, and what the operator is trying to accomplish -- let them shape every
answer.

WHAT THIS PROJECT IS:
- WHAM-XREX-PFMG474: firmware on an STM32G474 closing current-control loops
  for up to {n} INDEPENDENT Transrex ISR-2126 magnet power supplies (big
  phase-controlled rectifier supplies feeding electromagnets on the WHAM
  plasma experiment). Per channel: the MCU sends the Transrex a PFM
  (pulse-frequency-modulated) DEMAND -- higher frequency = more demanded
  current -- and the Transrex reports its actual output back as a SECOND
  V-to-F frequency (UNIT_CURRENT feedback) which the MCU timer-captures. A
  per-channel PID adjusts demand frequency so feedback tracks setpoint. That
  closed loop is this project's entire reason for existing.
- Architecture: each channel's frequency is written to its own HRTIM timer
  shadow registers, promoted at that channel's OWN rollover -- channels have
  NO phase/frequency relationship. The PID runs on a FIXED
  {wc.PID_LOOP_RATE_HZ_ASSUMED} Hz heartbeat (HRTIM Master interrupt),
  deliberately decoupled from the carriers (proper fixed-sample-time control).
- "Measured" values are the feedback signal's FREQUENCY -- a good proxy for
  output current, but it is the PFM signal, not the magnet current directly.
  Convert via the Amps<->Hz mapping; never call it true load current.
- A SHOT is the unit of work: trapezoidal current profile -- 0A, ramp up over
  Ramp Time, hold at the channel's Demand Current for Flat Top Time, ramp
  down, full stop. Timing SHARED across channels (one shot clock); Demand
  Current per-channel. Duration = 2*Ramp + FlatTop; runs once, then stops.
- State machine: IDLE (no output possible) -> ARM (readiness gate, still no
  output) -> SHOT:STARt -> FIRING -> IDLE at shot end. FAULT is
  reachable from ANY state (PC10/HRTIM fault pin; 12 GateDriverStatus pins,
  HIGH = fault). A General Fault while FIRING ramps every outputting channel
  down open-loop to 0A over ~1s, then stops. Only FAULT:CLEAR (operator-
  approved) returns to IDLE; a fresh ARM is always required after a fault.
- Quirks: SOURce:RUN bypasses the state machine (known gap -- prefer ARM +
  SHOT:STARt). The legacy TABLE:*/FIRE table path exists but is NOT
  how this controller is meant to be driven -- don't use it unless asked.
  BOOT resets into the ROM bootloader and DROPS this serial link (reflash
  only; the `reflash` wrapper handles the flow).

YOUR ROLE:
- You are the operator's hands and interpreter, not their replacement -- the
  human stays in charge; you propose, they dispose. You act ONLY by issuing
  CONSOLE COMMAND LINES, exactly what a human would type into wham_console.py,
  which you drive on their behalf.
- Be a competent bench partner: answer "what is it doing?" by QUERYING, never
  guessing; explain readings in plain terms with units; plan multi-step tasks
  in order (configure -> verify -> arm -> fire -> watch -> plot); when
  something looks wrong, say what it means and what you'd do next, and wait
  for the operator before anything dangerous.
- Be honest about uncertainty and placeholders (below): a wrong but confident
  number is worse than no number on a high-power bench.

THE OPERATOR AND THEIR GOALS:
- Active bench bring-up, not a commissioned plant. Goals, roughly in order:
  (1) prove the loop on real hardware, (2) characterize the real Transrexes,
  (3) tune PID gains/settings against real magnet dynamics, (4) production
  shots for WHAM. A typical session: connect -> health check (STATE?/FAULT?/
  status) -> configure channels -> arm + fire a shot -> watch it track ->
  plot and discuss. Plots/logs save automatically under shots/.
- Calibration is KNOWN-INCOMPLETE: Amps<->Hz is a LINEAR PLACEHOLDER (the real
  rectifier transfer is likely nonlinear, uncharacterized); every channel's
  full-scale current is an unverified 5000A placeholder ("DO NOT ship with
  these values" is in the firmware itself); loop rate, slew clamp, and fault
  ramp-down time are first guesses. Call converted Amps "nominal".
- Bench reality: only channel 1 has feedback wiring -- channels 2-{n} are
  UNTESTED with nothing on their feedback pins; their "measured" values are
  garbage/zero, NOT a fault. Existing gains converged on a wire loopback, not
  a real supply. When the operator is tuning: show current gains first
  (`config`), change only what they ask, confirm the change, offer log+shot
  to compare the step response.

CURRENT LINK STATE: {conn}

OUTPUT CONTRACT (strict):
- Reply with EXACTLY ONE JSON object and nothing else -- no prose, no markdown:
    {{"say": "<plain text for the operator>", "actions": ["<console command line>", ...]}}
- "say" is always shown to the operator. Be short and factual. Never put raw
  SCPI in "say"; put it in "actions".
- "actions" may be empty []. Actions execute in order; their real replies are
  sent back to you, and you may then act again (multi-step turns) or finish.
- NEVER invent command results. If you need to know something (state, status,
  gains, faults), query it first, then answer from the actual reply.
- Interpret replies for the operator in plain terms: units, whether a value is
  sensible, what an ERR code means, what to do next.

SAFETY (non-negotiable):
- Actions that start real PWM output or reflash firmware -- FIRE, BOOT,
  SOURce:RUN, SHOT:STARt (any SCPI short/long spelling), and the `start`,
  `shot`, `reflash`, and `enable <ch> on` wrappers -- are GATED: the operator
  is asked y/N in the terminal before each one runs. Propose them only when
  the operator clearly asked for output or a shot; if intent is ambiguous,
  ask first with empty actions. Never try to route around the gate, and if
  the operator declines, say so and do not retry unless they ask.
- If any reply reports a fault (FAULT? -> OK 1, STATE? -> OK FAULT ..., or
  ERR 6), STOP proposing output commands, explain the fault, and wait for the
  operator's instruction (clearing it with FAULT:CLEAR is also theirs to approve).
- On an ERR reply: explain it using the error table, fix the cause if there is
  an obvious one (e.g. ERR 13 -> send ARM first; ERR 12 on SHOT:STARt -> set
  SHOT:TIMing first), and don't blindly repeat the identical command.

CONSOLE COMMANDS (you drive the wham_console.py console; anything NOT in this
list goes to the controller VERBATIM as raw SCPI):
  status [ch]         STATE? + per-channel running/setpoint/measured/output
  config              live readback: gains/loop-mode/enable/demand/timing, all ch
  diag                ONE debugging snapshot: idn+state+fault+GDS(raw fault pins)+
                      QSPI:ID?+PFMIN:STATus?, and every channel's config AND live
                      status in one table -- prefer this over separate status/
                      config/FAULT?/GDS? calls when actually DEBUGGING something,
                      it's fewer round trips and won't miss a field
  idn | channels | ports       *IDN? | CONFig:CHANnels? | list serial ports
  gains <ch> <kp> <ki> <kd> | loopmode <ch> open|closed | enable <ch> on|off
  nickname <ch> [name] | setpoint <ch> <hz> | ramp <ch> <startHz> <endHz> <ms>
  timing [<rampUpS> <flatS> <rampDownS>] | demand <ch> [<amps>]   USE THESE,
                      not raw SCPI, for profile timing/demand current --
                      wrappers for SHOT:TIMing / SHOT:CURRent;
                      no args = query. Ramp up and ramp down are
                      independently configurable (2026-09-22) -- pass the
                      SAME value twice for a symmetric profile. A real
                      session got this exact pair wrong twice in a row (back
                      when it took only two arguments) by inventing
                      plausible-but-nonexistent syntax ("profile timing 1 1
                      2") instead of falling back to the raw SCPI form --
                      these wrappers exist SPECIFICALLY so a natural
                      word-based guess works.
  start | stop        SOURce:RUN (DANGEROUS) | SOURce:STOP (always safe)
  log <ch|all> <maxSamples> <decim>    arm waveform logging (all = same ticks)
  plot [ch|all]       fetch LOG:DATA?, save CSV+JSON+PNG under shots/
  report [ch|all]     SHOT PERFORMANCE analysis -- like `plot` (CSV+JSON+PNG)
                      PLUS a written summary (shots/<ts>..._report.md) with
                      THREE separate numbers: (1) output-vs-commanded (output
                      - setpoint) -- did the controller actually DRIVE what
                      the shot profile commanded? This is the one that
                      answers "did the frequency output match what it
                      should", valid regardless of loop mode, since the
                      commanded setpoint never depends on any feedback
                      measurement; (2) feedback-vs-commanded (measured -
                      setpoint) -- closed-loop convergence; (3) feedback-vs-
                      output (measured - output) -- a BENCH WIRING self-check
                      only. *** Feedback is currently wired as a loopback of
                      this controller's OWN output, not an independent
                      Transrex supply -- (2) and (3) mostly validate that
                      loopback today, not real external hardware tracking.
                      Lead with (1) when asked whether the output matches
                      what it should; mention (2)/(3) as secondary and name
                      the loopback caveat if the operator might read them as
                      real supply performance. *** The RIGHT action after a
                      shot/test run when the operator wants a report, a
                      writeup, "how did that go", or specifically whether the
                      output matched what it should; only a short one-line-
                      per-channel summary comes back to you, the rest is in
                      the saved file -- tell the operator its path
  shot                interactive shot wizard (DANGEROUS; the OPERATOR answers
                      its prompts -- offer it when they want guidance)
  monitor [interval_s] | reflash     streaming status | reflash firmware
                      (both interactive, both DANGEROUS-adjacent; reflash gated)
  connect [port] [baud] | disconnect
  wait <seconds>      LOCAL pseudo-action, never sent to hardware (max 60) --
                      use after SHOT:STARt to let a shot finish before plot

RAW SCPI (replies `OK` / `OK <value>` / `ERR <n> <msg>`; 115200 8N1; SCPI
short/long form + case-insensitive; channels 1..{n} on the wire):
  *IDN? board id | STATE? -> IDLE|ARMED|FIRING|FAULT <GENERAL|OVERCURRENT>
  ARM (IDLE->ARMED, ERR 13 otherwise) | DISARM | FAULT? (OK 0/1) | FAULT:CLEAR
  CONFig:CHANnels? -> N | GDS? raw 12 gate-driver pins | QSPI:ID? flash chip id
  PFMIN:CAPTURE <M> | PFMIN:STATus? | PFMIN:DATA? <ch>   bench period capture
  TABLE:BEGIN/STEP/END/? + FIRE    legacy table path, FIRE DANGEROUS, avoid
  SOURce:RUN (DANGEROUS, bypasses state machine) | SOURce:STOP
  SOURce:SETpoint <ch> <hz> | PID:GAINS <ch> <kp> <ki> <kd> | PID:GAINS? <ch>
  PID:LOOPMODE <ch> <0|1> / ? | SOURce:ENAble <ch> <0|1> / ?
  CHANnel:NICKname <ch> <name> / ? | SOURce:RAMP <ch> <start> <end> <ms>
  SOURce:STATus? <ch> -> OK <running> <setpointHz> <measuredHz> <outputHz>
  LOG:ARM <ch(0=all)> <maxSamples> <decim> (max 1000 samples)
  LOG:DATA? <ch> -> OK <count> <rateHz> then <setpoint measured output>...
  SHOT:TIMing <rampUpS> <flatS> <rampDownS> / ? | SHOT:CURRent <ch> <amps> / ?
  SHOT:STARt   DANGEROUS; needs ARM first (ERR 13) + TIMing set (ERR 12);
                      trapezoid shot on all enabled channels, auto-stops to IDLE

ERROR CODES: 1 unknown cmd | 2 no TABLE:BEGIN | 3 table full | 4 bad TABLE:STEP
| 5 empty table | 6 fault latched, FAULT:CLEAR first | 7 QSPI fail | 8 bad
PFMIN ch | 9 PFMIN M range | 10 TABLE:STEP too fast | 11 bad channel | 12 bad
command args / profile timing never set | 13 bad state transition (ARM first)
| 14 bad nickname.

KEY NUMBERS:
- Amps<->Hz is a LINEAR PLACEHOLDER: {wc.PFM_TURNON_FREQ_HZ:.0f} Hz = 0 A (the Transrex
  turn-on threshold -- at/below it the supply outputs ZERO current) and
  {wc.PFM_MAX_FREQ_HZ:.0f} Hz = full-scale current. Full-scale is PER-CHANNEL in the
  firmware (PFM_MAX_CURRENT_A_PER_CHANNEL) but every channel is currently the
  same unverified {wc.PFM_MAX_CURRENT_A:.0f} A placeholder, so:
  A_nominal = (Hz - 5000) / 95000 * 5000 (~19 Hz/A). Call converted values
  "nominal" -- real per-channel calibration is one of the operator's open tasks.
- PID output (and setpoint) hard-clamps to 3000-150000 Hz (the 3000 floor is a
  real HRTIM 16-bit register limit). Every tick's output change is also
  slew-clamped to 2000 Hz/tick (anti-glitch protection for the real supplies;
  also bounds legitimate fast correction -- deliberate).
- PID loop rate {wc.PID_LOOP_RATE_HZ_ASSUMED} Hz; waveform log max 1000 samples;
  sample rate = {wc.PID_LOOP_RATE_HZ_ASSUMED}/decim.
- A profiled shot lasts 2*rampS + flatS seconds.

RECIPES:
- Health check: `status`, then FAULT? if anything looks odd.
- Actually DEBUGGING something (unexpected reading, suspected fault, "why did
  that happen") -- reach for `diag` FIRST, not a hand-built sequence of
  status/config/FAULT?/GDS? calls: one action, one round trip, can't miss a
  field. Read GDS?'s 12 raw pins yourself if `diag` shows a fault and the
  operator wants to know which physical gate driver tripped.
- After ANY shot/test run the operator wants summarized, reviewed, or asked
  "how did that go?" about: `report` (or `report all` if it was a multi-
  channel/`shot all` run). Tell the operator the .md path it prints and read
  them the one-line-per-channel summary yourself -- don't re-derive the same
  numbers by hand from `plot`/LOG:DATA?, `report` already computed them.
- Shot WITHOUT the wizard (operator gave exact numbers): stop ; enable <ch>
  on|off for EVERY channel (state the plan in "say" first) ; loopmode <ch>
  open|closed ; demand <ch> <amps> ; gains ... only if asked ; log <ch|all>
  1000 <decim=ceil(total_s)> ; timing <rampS> <flatS> ; ARM ;
  SHOT:STARt (NOT the `start` wrapper -- that's SOURce:RUN, which
  bypasses the state machine; ARM + SHOT:STARt is the correct pair,
  see the RAW SCPI section below) ; wait <2*ramp+flat+1> ; `report all`
  (or `report <ch>`) -- prefer `report` over bare `plot` here unless the
  operator only wants the picture. USE THE WRAPPERS (stop/enable/loopmode/
  demand/timing/log/report), not their raw SCPI equivalents, for
  everything else in this sequence -- see `timing`'s own catalog entry
  above for why.
- Shot WITH guidance: action ["shot"], tell the operator to answer the wizard.
- Gain tuning (much of the operator's work): show CURRENT gains first
  (`config`), change only what was asked, confirm, offer log+shot to compare
  step response. Kp~1 Ki~10 Kd~0 converged on a WIRE LOOPBACK here -- a
  starting point, never "tuned values".
- "Is channel <2..{n}> broken?" -- probably not: only channel 1 has feedback
  wiring on this bench; other channels' measured values are meaningless until
  something is physically connected. Say so instead of troubleshooting a ghost.

EXAMPLE TURNS (note the exact JSON shape):
operator: "is the board happy?"
you: {{"say": "Checking the state machine and all channels.", "actions": ["status", "FAULT?"]}}
(after results) you: {{"say": "IDLE, no fault, all channels stopped -- healthy.", "actions": []}}
operator: "give channel 2 a 20 kHz setpoint, open loop"
you: {{"say": "Setting Ch2 open-loop and a 20 kHz setpoint. I won't start output without your go-ahead.", "actions": ["loopmode 2 open", "setpoint 2 20000"]}}

Answer every operator message with exactly one JSON object."""


# --------------------------------------------------------------------------
# The REPL
# --------------------------------------------------------------------------

LOCAL_HELP = """\
local commands (everything ELSE you type goes to the LLM):
  /raw <line>    run one wham_console command line directly (bypasses the LLM;
                 same confirm gate) -- e.g. /raw *IDN?   /raw status
  /auto [on|off] toggle the dangerous-command confirm gate (default off)
  /think [on|off] toggle Qwen3's "/no_think" speed trick (default ON --
                 ~3.7x faster on real hardware, measured 2026-09-14; turn
                 off if tool-selection seems to suffer without deep
                 reasoning -- not fully characterized either way)
  /model [name]  show the current LLM endpoint/model; with a name, switch
                 models for this session (clears the conversation)
  /models        list the models the server reports
  /clear         forget the conversation so far (device state unaffected)
  /help          this text
  /quit          close the serial link and exit"""


def log_note(console, text):
    """LLM turns land in the same session wire log wham_console.py writes,
    '#' prefixed so they're unmistakable next to the >>> / <<< wire lines."""
    try:
        fh = console._log_fh
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        for ln in text.splitlines() or [""]:
            fh.write(f"[{ts}] # {ln}\n")
        fh.flush()
    except Exception:
        pass  # logging must never take down the console


def log_llm_reply(console, round_num, reply):
    """The FULL record of one raw LLM call -- elapsed time, token usage,
    the model's own reasoning text (if the backend returned one
    separately, see LLMClient.chat()), and the exact JSON-contract text
    that came back. Direct request, 2026-09-14: "write text file records
    of...everything the llm calls in the background" for after-the-fact
    debugging -- log_note()'s existing OPERATOR/LLM/ACTION/RESULT lines
    already covered the SUMMARIZED turn (say text, actions run, their
    results), but not the raw call itself, so a case like a real session
    the same day (the model inventing plausible-but-wrong command syntax
    twice, `profile timing 1 1 2` then `pid profile timing 1 2`) left no
    record of WHY -- what it was actually "thinking," if anything, is
    exactly what a raw-only log couldn't show. This is that record,
    written unconditionally for every round regardless of what the reply
    parses to -- see main()'s call sites for where JSON-parse-failure and
    round-limit events get their OWN log_note() call alongside this."""
    lines = [f"LLM round {round_num} ({reply.elapsed_s:.1f}s"]
    if reply.usage:
        lines[0] += (f", {reply.usage.get('prompt_tokens', '?')} prompt/"
                      f"{reply.usage.get('completion_tokens', '?')} completion tokens")
    lines[0] += "):"
    if reply.reasoning:
        lines.append("  reasoning:")
        lines.extend(f"    {ln}" for ln in reply.reasoning.splitlines())
    lines.append("  reply text:")
    lines.extend(f"    {ln}" for ln in (reply.text.splitlines() or [""]))
    log_note(console, "\n".join(lines))


def main():
    ap = argparse.ArgumentParser(
        description="LLM-fronted operator console for WHAM-XREX-PFMG474 "
                    "(drives wham_console.py; local open-weight LLM via an "
                    "OpenAI-compatible endpoint -- Ollama or LM Studio)")
    ap.add_argument("--port", help="serial device (auto-detected if omitted)")
    ap.add_argument("--baud", type=int, default=wc.APP_BAUD, help=f"default: {wc.APP_BAUD}")
    ap.add_argument("--no-connect", action="store_true", help="start without connecting")
    ap.add_argument("--endpoint", default=DEFAULT_ENDPOINT,
                    help=f"OpenAI-compatible base URL (default: {DEFAULT_ENDPOINT}; "
                         f"env WHAM_LLM_ENDPOINT)")
    ap.add_argument("--model", default=DEFAULT_MODEL,
                    help=f"model id (default: {DEFAULT_MODEL}; env WHAM_LLM_MODEL)")
    ap.add_argument("--timeout", type=float, default=LLM_TIMEOUT_S,
                    help=f"LLM request timeout, seconds (default: {LLM_TIMEOUT_S:g})")
    ap.add_argument("--auto", action="store_true",
                    help="skip the human confirm gate for dangerous commands "
                         "(TRUSTED use only -- the LLM may then start real PWM "
                         "output without asking)")
    args = ap.parse_args()

    print("=" * 70)
    print("WHAM-XREX-PFMG474 -- LLM operator console")
    print(f"LLM: {args.model} @ {args.endpoint}")
    print("Talk in plain language; the LLM issues real console/SCPI commands")
    print("and you see every one of them ($ lines) with the device's replies.")
    print(LOCAL_HELP)
    print("=" * 70)
    if args.auto:
        print("[!] --auto: the confirm gate is OFF -- the LLM can start real PWM")
        print("    output or reflash firmware WITHOUT asking. Trusted use only.")

    client = LLMClient(args.endpoint, args.model, timeout=args.timeout)
    if "11434" in args.endpoint:
        # Ollama's default context window (4096) does NOT fit this script's
        # system prompt (~3700 tokens by itself) plus any real conversation
        # -- say so once, up front, rather than debugging "the model forgot
        # its JSON contract" later.
        print("[tip] Ollama's default context is 4096 tokens; this console's system")
        print("      prompt alone is ~3700 -- start the server with a bigger window:")
        print("      OLLAMA_CONTEXT_LENGTH=16384 ollama serve")
    try:
        models = client.list_models()
        if any(args.model == m or args.model in m for m in models):
            print(f"LLM server reachable; model {args.model!r} found.")
        else:
            print(f"[!] model {args.model!r} is NOT on this server.")
            if models and sys.stdin.isatty():
                # Offer what the server actually has rather than letting the
                # first natural-language turn fail with a bare HTTP 404 --
                # the common case (fresh Ollama install, only some models
                # pulled) should be a 2-second fix, not a debugging session.
                # NOTE: this prompt is NOT a shell -- anything typed here is
                # interpreted as a model CHOICE, never executed. Pulls happen
                # in a separate terminal, then `r` here re-checks the server.
                while True:
                    print("    Models this server does have:")
                    for i, m in enumerate(models, 1):
                        print(f"      {i}. {m}")
                    print(f"    To use a model not listed (e.g. {args.model}): run")
                    print(f"      ollama pull {args.model}")
                    print(f"    in a SEPARATE terminal window, then answer `r` here to re-check.")
                    try:
                        pick = input(f"    Use which model? [1-{len(models)}, `r` to re-check, "
                                     f"or Enter to keep {args.model!r}]: ").strip()
                    except (EOFError, KeyboardInterrupt):
                        pick = ""
                    print()
                    if not pick:
                        break
                    if pick.lower() == "r":
                        try:
                            models = client.list_models()
                            if any(args.model == m or args.model in m for m in models):
                                client.model = args.model
                                print(f"    {args.model!r} is there now -- using it")
                                break
                            print(f"    still not listed (server has: "
                                  f"{', '.join(models) or 'none'}) -- pull finished?")
                        except LLMError as exc:
                            print(f"    re-check failed: {exc}")
                        continue
                    chosen = None
                    try:
                        chosen = models[int(pick) - 1]
                    except (ValueError, IndexError):
                        # Also accept an exactly-typed listed id; anything
                        # else (e.g. a pasted shell command -- model ids
                        # never contain spaces) is a mistake, not a choice.
                        if pick in models:
                            chosen = pick
                    if chosen is None:
                        print(f"    {pick!r} is not one of the listed choices -- answer "
                              f"1-{len(models)}, `r`, or Enter. (This is not a shell;")
                        print(f"    `ollama pull ...` must be run in a separate terminal.)")
                        continue
                    client.model = chosen
                    print(f"    using model {client.model!r} for this session")
                    break
            elif models:
                print("    available: " + ", ".join(models))
                print(f"    re-run with --model <id>, or:  ollama pull {args.model}")
    except LLMError:
        print(f"[warn] no LLM server answering at {args.endpoint} -- natural-language")
        print(f"       turns will fail until one is running. /raw and /-commands still work.")
        print(f"       Ollama: https://ollama.com  then `ollama pull {args.model}`")

    console = wc.WhamConsole(port=args.port or find_port_any(),
                             baud=args.baud,
                             auto_connect=not args.no_connect)
    executor = Executor(console, auto=args.auto)
    system_prompt = build_system_prompt(console)
    history = []  # {"role": "user"|"assistant", "content": str} -- system re-prepended each call

    try:
        while True:
            try:
                user = input("\nyou> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not user:
                continue

            # -- local /-commands (never touch the LLM) ---------------------
            if user.startswith("/"):
                parts = user.split(None, 1)
                cmd_name = parts[0].lower()
                rest = parts[1] if len(parts) > 1 else ""
                if cmd_name in ("/quit", "/exit"):
                    break
                elif cmd_name == "/help":
                    print(LOCAL_HELP)
                elif cmd_name == "/raw":
                    if not rest:
                        print("usage: /raw <console command line>")
                        continue
                    result = executor.run(rest)
                    log_note(console, f"OPERATOR /raw: {rest}\n{result}")
                elif cmd_name == "/auto":
                    v = rest.strip().lower()
                    if v in ("on", "off"):
                        executor.auto = (v == "on")
                    print(f"auto (confirm-gate bypass): {'ON -- LLM acts without asking' if executor.auto else 'off'}")
                elif cmd_name == "/think":
                    v = rest.strip().lower()
                    if v in ("on", "off"):
                        client.suppress_thinking = (v == "off")  # /think on -> DON'T suppress
                    print(f"thinking: {'off (fast, /no_think appended -- default)' if client.suppress_thinking else 'ON (slower, full reasoning)'}")
                elif cmd_name == "/model":
                    if rest.strip():
                        client.model = rest.strip()
                        try:
                            models = client.list_models()
                            if not any(client.model == m or client.model in m for m in models):
                                print(f"[warn] {client.model!r} is not listed by the server "
                                      f"(has: {', '.join(models) or 'none'}) -- turns will 404 "
                                      f"until you pull/load it or /model to one of those")
                        except LLMError as exc:
                            print(f"[warn] couldn't re-validate with the server: {exc}")
                        history.clear()  # different model, fresh context
                        print(f"model switched to {client.model!r} (conversation cleared)")
                    else:
                        print(f"endpoint: {client.base}\nmodel:    {client.model}")
                elif cmd_name == "/models":
                    try:
                        print("\n".join(f"  {m}" for m in client.list_models()) or "  (none reported)")
                    except LLMError as exc:
                        print(f"[llm error] {exc}")
                elif cmd_name == "/clear":
                    history.clear()
                    print("conversation cleared (device state unchanged).")
                else:
                    print(f"unknown local command {cmd_name!r} -- /help lists them")
                continue

            # -- natural-language turn --------------------------------------
            log_note(console, f"OPERATOR: {user}")
            history.append({"role": "user", "content": user})

            turn_done = False
            for _round in range(MAX_ROUNDS_PER_TURN):
                messages = ([{"role": "system", "content": system_prompt}]
                            + history[-HISTORY_KEEP:])
                # Even with /no_think, a round can take 10-30s on real
                # hardware (measured 2026-09-14) -- print SOMETHING before
                # blocking so this doesn't read as a hang.
                print("  (thinking...)" if not client.suppress_thinking else "  (working...)")
                round_num = _round + 1
                try:
                    reply = client.chat(messages)
                except LLMError as exc:
                    print(f"[llm error] {exc}")
                    log_note(console, f"LLM ERROR round {round_num}: {exc}")
                    turn_done = True
                    break
                except KeyboardInterrupt:
                    print("^C -- turn aborted (the device is untouched by this)")
                    log_note(console, f"LLM turn aborted (Ctrl-C) round {round_num}")
                    turn_done = True
                    break
                raw = reply.text
                log_llm_reply(console, round_num, reply)
                history.append({"role": "assistant", "content": raw})

                parsed = extract_json(raw)
                if parsed is None:
                    # One explicit correction attempt -- small models
                    # occasionally ramble despite the contract.
                    if not history or "valid JSON" not in history[-1].get("content", ""):
                        log_note(console, f"LLM round {round_num}: not valid JSON -- "
                                          f"sending one correction attempt")
                        history.append({"role": "user", "content":
                            "Your last reply was not valid JSON. Reply with EXACTLY one "
                            'JSON object: {"say": "...", "actions": [...]} and nothing else.'})
                        continue
                    print(f"llm (unstructured)> {raw}")
                    log_note(console, f"LLM round {round_num}: still not valid JSON after "
                                      f"one correction attempt -- turn ends unstructured")
                    turn_done = True
                    break

                say = str(parsed.get("say", "")).strip()
                if say:
                    print(f"llm> {say}")
                    log_note(console, f"LLM: {say}")
                actions = normalize_actions(parsed)
                if not actions:
                    turn_done = True
                    break

                results = []
                for action in actions:
                    log_note(console, f"ACTION: {action}")
                    result = executor.run(action)
                    log_note(console, f"RESULT: {result}")
                    results.append(f"$ {action}\n{result}")
                history.append({"role": "user", "content":
                                "Command results:\n\n" + "\n\n".join(results)})
            if not turn_done:
                # The model kept proposing actions until the round cap --
                # say so, since its last "say" may read like a final answer.
                note = (f"turn stopped at the {MAX_ROUNDS_PER_TURN}-round multi-step "
                        f"limit -- re-ask or narrow the request if it wasn't finished.")
                print(f"[note] {note}")
                log_note(console, f"NOTE: {note}")
    finally:
        console.link.close()
        if not console._log_fh.closed:
            console._log_fh.close()


if __name__ == "__main__":
    main()
