"use strict";

import React from 'react';
import ReactDOM from 'react-dom';
import { BrowserRouter } from 'react-router-dom'
import App from './App';
import WebFontLoader from 'webfontloader';
//import 'react-md/dist/react-md.indigo-pink.min.css'
import '../node_modules/bootstrap/dist/css/bootstrap.min.css';
import './scss/custom.css';

WebFontLoader.load({
    google: {
        families: ['Roboto:300,400,500,700', 'Material Icons'],
    },
});

ReactDOM.render(<BrowserRouter><App /></BrowserRouter>, document.getElementById('app'));
