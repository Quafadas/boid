A naive (vibe coded!) implementation of https://eater.net/boids

# To Run

In a terminal

```sh

cs launch io.github.quafadas:sjsls_3:0.2.6

```

# WASM vs JS

It seems to be more jittery on WASM, and when increasing the number of boids WASM cracks first.

I couldn't get it to work on Safari. Worked for me in Chrome enabling experimental WebAssembly features, as described here

https://www.scala-js.org/news/2025/04/21/announcing-scalajs-1.19.0/

Question: what is defined as "JS interop" in scalaJS. As this is rather mathematical, I had hoped, that JS would crack first.



