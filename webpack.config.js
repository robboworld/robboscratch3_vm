const CopyWebpackPlugin = require('copy-webpack-plugin');
const defaultsDeep = require('lodash.defaultsdeep');
const path = require('path');
const UglifyJsPlugin = require('uglifyjs-webpack-plugin');

const isProduction = process.env.NODE_ENV === 'production';
const isAppBuild = process.env.BUILD_MODE === 'app';
const useBabelCache = !isProduction;

const babelLoaderOptions = {
    presets: [['@babel/preset-env', {targets: {browsers: ['last 3 versions', 'Safari >= 8', 'iOS >= 8']}}]]
};

const babelRule = {
    test: /\.js$/,
    include: path.resolve(__dirname, 'src'),
    use: useBabelCache ? [
        {
            loader: 'cache-loader',
            options: {
                cacheDirectory: path.resolve(__dirname, 'node_modules/.cache/babel-loader')
            }
        },
        {
            loader: 'babel-loader',
            options: babelLoaderOptions
        }
    ] : {
        loader: 'babel-loader',
        options: babelLoaderOptions
    }
};

const base = {
    mode: isProduction ? 'production' : 'development',
    devServer: {
        contentBase: false,
        host: '0.0.0.0',
        port: process.env.PORT || 8073
    },
    devtool: 'cheap-module-source-map',
    output: {
        library: 'VirtualMachine',
        filename: '[name].js'
    },
    module: {
        rules: [
            babelRule,
            {
                test: /\.mp3$/,
                loader: 'file-loader'
            }
        ]
    },
    optimization: {
        minimizer: [
            new UglifyJsPlugin({
                include: /\.min\.js$/
            })
        ]
    },
    plugins: []
};

const webAndNode = [
    defaultsDeep({}, base, {
        target: 'web',
        entry: {
            'scratch-vm': './src/index.js',
            'scratch-vm.min': './src/index.js'
        },
        output: {
            libraryTarget: 'umd',
            path: path.resolve('dist', 'web')
        },
        module: {
            rules: base.module.rules.concat([
                {
                    test: require.resolve('./src/index.js'),
                    loader: 'expose-loader?VirtualMachine'
                }
            ])
        }
    }),
    defaultsDeep({}, base, {
        target: 'node',
        entry: {
            'scratch-vm': './src/index.js'
        },
        output: {
            libraryTarget: 'commonjs2',
            path: path.resolve('dist', 'node')
        },
        externals: {
            'decode-html': true,
            'escape-html': true,
            'format-message': true,
            'htmlparser2': true,
            'immutable': true,
            'jszip': true,
            'minilog': true,
            'nets': true,
            'scratch-parser': true,
            'socket.io-client': true,
            'text-encoding': true
        }
    })
];

const playground = defaultsDeep({}, base, {
    target: 'web',
    entry: {
        'scratch-vm': './src/index.js',
        'vendor': [
            'stats.js/build/stats.min.js',
            'scratch-blocks/dist/vertical.js',
            'scratch-audio',
            'scratch-storage',
            'scratch-render'
        ],
        'video-sensing-extension-debug': './src/extensions/scratch3_video_sensing/debug'
    },
    output: {
        path: path.resolve(__dirname, 'playground'),
        filename: '[name].js'
    },
    module: {
        rules: base.module.rules.concat([
            {
                test: require.resolve('./src/index.js'),
                loader: 'expose-loader?VirtualMachine'
            },
            {
                test: require.resolve('./src/extensions/scratch3_video_sensing/debug.js'),
                loader: 'expose-loader?Scratch3VideoSensingDebug'
            },
            {
                test: require.resolve('stats.js/build/stats.min.js'),
                loader: 'script-loader'
            },
            {
                test: require.resolve('scratch-blocks/dist/vertical.js'),
                loader: 'expose-loader?Blockly'
            },
            {
                test: require.resolve('scratch-audio/src/index.js'),
                loader: 'expose-loader?AudioEngine'
            },
            {
                test: require.resolve('scratch-storage/src/index.js'),
                loader: 'expose-loader?ScratchStorage'
            },
            {
                test: require.resolve('scratch-render/src/index.js'),
                loader: 'expose-loader?ScratchRender'
            }
        ])
    },
    performance: {
        hints: false
    },
    plugins: base.plugins.concat([
        new CopyWebpackPlugin([{
            from: 'node_modules/scratch-blocks/media',
            to: 'media'
        }, {
            from: 'node_modules/scratch-storage/dist/web'
        }, {
            from: 'node_modules/scratch-render/dist/web'
        }, {
            from: 'node_modules/scratch-svg-renderer/dist/web'
        }, {
            from: 'src/playground'
        }])
    ])
});

module.exports = isAppBuild ? webAndNode : webAndNode.concat([playground]);
