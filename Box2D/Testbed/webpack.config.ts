import * as path from "path";
import * as webpack from "webpack";
// import * as CleanWebpackPlugin from "clean-webpack-plugin";
import * as CleanWebpackPlugin from 'clean-webpack-plugin';

const distDir = path.resolve(__dirname, "../../dist/Testbed");

const config: webpack.Configuration = {
  mode: "production",
  entry: "./Testbed.ts",
  resolve: {
      extensions: [".ts"],
  },
  output: {
      path: distDir,
      filename: "[name]_bundle.js",
  },
  module: {
      rules: [
          {
              test: /\.ts$/,
              loaders: ["ts-loader"],
          },
      ],
  },

  plugins: [
      new CleanWebpackPlugin([distDir], {
          root: path.resolve(__dirname, "../.."),
      }),
  ],
};

export default config;
