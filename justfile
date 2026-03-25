setup-ide:
  scala-cli setup-ide .

build:
  scala-cli package . --power --output target/ -f

buildJs:
  scala-cli package . --power --output targetJs/ -f

dev:
  cs launch io.github.quafadas:sjsls_3:0.2.8 -- --path-to-index-html assets/

trace:
  cs launch io.github.quafadas:sjsls_3:0.2.5-11-ace5b0 -- --log-level trace