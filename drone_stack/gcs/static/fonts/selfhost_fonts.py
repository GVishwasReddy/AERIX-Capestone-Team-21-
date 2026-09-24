"""Self-host the GCS webfonts.

The GCS is flown in the field, where the operator's laptop is associated with
the Pi's own network and has no route to the internet. Pulling fonts from
fonts.googleapis.com meant the UI silently fell back to system faces exactly
when it mattered - and a font swap changes every metric in a scaled layout, so
the field build looked different from the bench build. These are downloaded
once and served from /static/fonts, so the UI is byte-identical offline.

Latin subset only: the GCS has no non-latin strings and the other 13 subsets
are dead weight on a 2.4 GHz link.
"""
import io, os, re, urllib.request

UA = {"User-Agent": "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7) "
                    "AppleWebKit/537.36 (KHTML, like Gecko) Chrome/120.0 Safari/537.36"}
src = io.open("gf.css", encoding="utf-8").read()

# Google emits "/* latin */" immediately before each @font-face it applies to.
blocks = re.findall(r"/\*\s*([a-z0-9-]+)\s*\*/\s*(@font-face\s*\{[^}]*\})", src)
out, kept = [], 0
for subset, block in blocks:
    if subset != "latin":
        continue
    fam = re.search(r"font-family:\s*'([^']+)'", block).group(1)
    wght = re.search(r"font-weight:\s*(\d+)", block).group(1)
    style = re.search(r"font-style:\s*(\w+)", block).group(1)
    url = re.search(r"url\((https://[^)]+\.woff2)\)", block).group(1)
    name = "%s-%s%s.woff2" % (fam.replace(" ", ""), wght, "i" if style == "italic" else "")
    if not os.path.exists(name):
        req = urllib.request.Request(url, headers=UA)
        data = urllib.request.urlopen(req, timeout=30).read()
        io.open(name, "wb").write(data)
    kept += 1
    out.append(
        "@font-face{font-family:'%s';font-style:%s;font-weight:%s;font-display:swap;"
        "src:url('/static/fonts/%s') format('woff2');"
        "unicode-range:U+0000-00FF,U+0131,U+0152-0153,U+02BB-02BC,U+02C6,U+02DA,"
        "U+02DC,U+0304,U+0308,U+0329,U+2000-206F,U+20AC,U+2122,U+2191,U+2193,"
        "U+2212,U+2215,U+FEFF,U+FFFD;}" % (fam, style, wght, name)
    )

io.open("fonts.css", "w", encoding="utf-8").write(
    "/* Self-hosted so the GCS renders identically with no internet - see\n"
    "   selfhost_fonts.py. Regenerate by re-running that script. */\n"
    + "\n".join(out) + "\n")
os.remove("gf.css")
print("faces kept:", kept)
print("files:", sorted(f for f in os.listdir(".") if f.endswith(".woff2")))
print("total KB:", round(sum(os.path.getsize(f) for f in os.listdir(".")
                             if f.endswith(".woff2")) / 1024, 1))
