setup-ide:
  scala-cli setup-ide .

dev:
  cs launch io.github.quafadas:sjsls_3:0.2.6

trace:
  cs launch io.github.quafadas:sjsls_3:0.2.5-11-ace5b0 -- --log-level trace