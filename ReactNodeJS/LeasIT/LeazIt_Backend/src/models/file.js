"use strict";

const mongoose = require('mongoose');

const FileSchema  = new mongoose.Schema({

    name: {
        type: String,
       // required: true,
    },
});

FileSchema.set('versionKey', false);

// Export the Movie model
module.exports = mongoose.model('File', FileSchema);