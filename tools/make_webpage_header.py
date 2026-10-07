#!/usr/bin/env python3
"""speed_tracer_pc.html から、ファームウェアに埋め込む webpage_html.h を作る。

使い方: python3 tools/make_webpage_header.py speed_tracer_pc.html firmware/speed_tracer_production/webpage_html.h

ESP32が http://192.168.4.1/ で配信するページは、常にこのリポジトリの speed_tracer_pc.html と同じ中身になる
(手作業でコピーして古い版を埋め込んでしまう事故を防ぐ)。
"""
import sys

DELIM = "rawliteral"


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__, file=sys.stderr)
        return 2
    src, dst = sys.argv[1], sys.argv[2]
    with open(src, encoding="utf-8", newline="") as f:
        html = f.read()
    if ")" + DELIM + '"' in html:
        print(f'{src} に )' + DELIM + '" が含まれているため、生文字列として埋め込めません', file=sys.stderr)
        return 1
    if "</html>" not in html:
        print(f"{src} がHTMLとして完結していません(</html> がありません)", file=sys.stderr)
        return 1
    out = (
        "// このファイルは speed_tracer_pc.html の内容をそのままC++のraw文字列として埋め込んだものです。\n"
        "#pragma once\n"
        'const char INDEX_HTML[] PROGMEM = R"' + DELIM + "(\n" + html + ")" + DELIM + '";\n'
    )
    with open(dst, "w", encoding="utf-8", newline="") as f:
        f.write(out)
    print(f"{dst}: {len(out.encode('utf-8'))} bytes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
