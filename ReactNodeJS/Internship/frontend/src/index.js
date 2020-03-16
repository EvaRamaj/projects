"use strict";

import React from 'react';
import ReactDOM from 'react-dom';

import App from './App';
import WebFontLoader from 'webfontloader';
import './scss/custom.css'
import 'react-md/dist/react-md.indigo-pink.min.css'

WebFontLoader.load({
    google: {
        families: ['Roboto:300,400,500,700', 'Material Icons', 'Gothic A1'],
    },
});

ReactDOM.render(<App />, document.getElementById('app'));
