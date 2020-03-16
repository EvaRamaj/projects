"use strict";

const express    = require('express');
const bodyParser = require('body-parser');
const helmet     = require('helmet');

const middlewares = require('./middlewares');

const auth  = require('./routes/auth');
const movie = require('./routes/movie');
const user = require('./routes/user');
const conversation = require('./routes/conversation');
const message = require('./routes/message');
const item = require('./routes/item');
const booking = require('./routes/booking');
const itemEvaluation = require('./routes/itemEvaluation');
const userEvaluation = require('./routes/userEvaluation');
const category = require('./routes/category');
const photo = require('./routes/photo');
const file = require('./routes/file');

const api = express();


// Adding Basic Middlewares
api.use(helmet());
api.use(bodyParser.json());
api.use(bodyParser.urlencoded({ extended: false }));
api.use(middlewares.allowCrossDomain);


// Basic route
api.get('/', (req, res) => {
    res.json({
        name: 'LeazIt Backend'
    });
});


// API routes
api.use('/auth'  , auth);
api.use('/movies', movie);
api.use('/users', user);
api.use('/users/:id/messages', message);
api.use('/chats', conversation);
api.use('/items', item);
api.use('/bookings', booking);
api.use('/item_evaluations', itemEvaluation);
api.use('/user_evaluations', userEvaluation);
api.use('/categories', category);
api.use('/photos', photo);
api.use('/files', file);



module.exports = api;