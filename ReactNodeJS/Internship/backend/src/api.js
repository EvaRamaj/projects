"use strict";

const express    = require('express');
const bodyParser = require('body-parser');
const helmet     = require('helmet');

const middlewares = require('./middlewares');

const auth  = require('./routes/auth');
const company = require('./routes/company');
const job = require('./routes/job');
const user = require('./routes/user');
const jobApplication = require('./routes/jobApplication');
const imageUpload = require('./routes/uploadImage');
const project = require('./routes/project');

const api = express();

// Adding Basic Middlewares
api.use(helmet());
api.use(bodyParser.json());
api.use(bodyParser.urlencoded({ extended: false }));
api.use(middlewares.allowCrossDomain);


// Basic route
api.get('/', (req, res) => {
    res.json({
        name: 'team WAYER'
    });
});

// API routes
api.use('/auth'  , auth);
api.use('/job', job);
api.use('/company', company);
api.use('/user', user);
api.use('/application', jobApplication);
api.use('/image',imageUpload);
api.use('/project',project);

module.exports = api;
