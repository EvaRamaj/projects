"use strict";

const mongoose = require('mongoose');

const PhotoSchema  = new mongoose.Schema({

    name: {
        type: String,
       // required: true,
    },
});

PhotoSchema.set('versionKey', false);

// Export the Movie model
module.exports = mongoose.model('Photo', PhotoSchema);