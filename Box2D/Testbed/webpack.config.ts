import * as path from 'path';
import * as webpack from 'webpack';
const CleanWebpackPlugin = require('clean-webpack-plugin');


const distDir = path.resolve(__dirname, '../dist');

const config: webpack.Configuration = {
  mode: 'production',
  entry: './Testbed.ts',
  resolve: {
      extensions: ['.ts']
  },
  output: {
      path: distDir,
      filename: '[name]_bundle.js'
  },
  module: {
      rules: [
          {
              test: /\.ts$/,
              loaders: ['ts-loader']
          }
      ]
  },

  plugins: [
      new CleanWebpackPlugin([distDir]),
  ]
};

export default config;
