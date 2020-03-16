"use strict";

const mongoose = require('mongoose');

// Define the movie schema

const JobApplicationSchema  = new mongoose.Schema({
    message: String,
    jobId: {type: mongoose.Schema.Types.ObjectId, ref: 'Job'},
    userId: {type: mongoose.Schema.Types.ObjectId, ref: 'UserProfile'}
});

JobApplicationSchema.set('timestamps', true);

// Export the Person model
module.exports = mongoose.model('JobApplication', JobApplicationSchema);
