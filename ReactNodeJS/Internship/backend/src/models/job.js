"use strict";

const mongoose = require('mongoose');

// Define the movie schema

const JobsSchema  = new mongoose.Schema({
    title: {
        type: String,
        required: true
    },
    description: String,
    salary: Number,
    companyId: {type: mongoose.Schema.Types.ObjectId, ref: 'Company'}
});

JobsSchema.set('timestamps', true);

// Export the Jobs Model
module.exports = mongoose.model('Job', JobsSchema);
