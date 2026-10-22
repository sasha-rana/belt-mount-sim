import { defineConfig } from "vite";

// Relative asset paths, so the build works at any URL prefix
// (e.g. https://sasha-rana.github.io/belt-mount-sim/).
export default defineConfig({
  base: "./",
  build: { chunkSizeWarningLimit: 1000 },
});
