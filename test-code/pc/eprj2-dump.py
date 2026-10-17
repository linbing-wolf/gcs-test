# 嘉立创EDA 专业版工程 (.eprj2) 其实是个 SQLite 数据库。
# 用法：
#   python eprj2-dump.py <db>                         列出所有表
#   python eprj2-dump.py <db> --schema <表名>          看这张表的建表语句
#   python eprj2-dump.py <db> --sql "select ..."       直接跑一条 SQL
#   python eprj2-dump.py <db> --find 云台               在所有 TEXT 列里搜关键词
import sqlite3
import sys


def main(argv):
    db = argv[1]
    con = sqlite3.connect(db)
    cur = con.cursor()

    if len(argv) == 2:
        cur.execute("select name from sqlite_master where type='table' order by name")
        names = [r[0] for r in cur.fetchall()]
        print(len(names), "tables")
        for n in names:
            print(n)
        return

    mode = argv[2]
    if mode == "--schema":
        cur.execute("select sql from sqlite_master where name=?", (argv[3],))
        row = cur.fetchone()
        print(row[0] if row else "(no such table)")
        return

    if mode == "--sql":
        cur.execute(argv[3])
        for row in cur.fetchall():
            print(row)
        return

    if mode == "--find":
        need = argv[3]
        cur.execute("select name from sqlite_master where type='table' order by name")
        for (tname,) in cur.fetchall():
            try:
                cur.execute("pragma table_info(%s)" % tname)
                cols = [c[1] for c in cur.fetchall()]
                if not cols:
                    continue
                where = " or ".join("cast(%s as text) like ?" % c for c in cols)
                cur.execute("select * from %s where %s" % (tname, where),
                            tuple(["%" + need + "%"] * len(cols)))
                for row in cur.fetchall():
                    print("=== %s ===" % tname)
                    for c, v in zip(cols, row):
                        s = str(v)
                        if len(s) > 400:
                            s = s[:400] + "..."
                        print("  %s = %s" % (c, s))
            except sqlite3.Error as exc:
                print("(%s: %s)" % (tname, exc))
        return

    if mode == "--decode":
        # history_data 里的 dataStr 是 base64（解出来多半还是压缩的），
        # 解完写到 out 文件，方便直接 rg 找丝印文字。
        import base64
        import gzip
        import zlib
        row_id = int(argv[3])
        out = argv[4]
        cur.execute("select dataStr from history_data where id=?", (row_id,))
        raw = base64.b64decode(cur.fetchone()[0])
        head = raw[:4].hex()
        data = raw
        if raw[:2] == b"\x1f\x8b":
            data = gzip.decompress(raw)
        elif raw[:1] == b"\x78":
            data = zlib.decompress(raw)
        with open(out, "wb") as fh:
            fh.write(data)
        print("id=%d raw=%d head=%s out=%d" % (row_id, len(raw), head, len(data)))
        return

    print("unknown mode", mode)


if __name__ == "__main__":
    main(sys.argv)
