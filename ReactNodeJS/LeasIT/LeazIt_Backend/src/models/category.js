"use strict";

const mongoose = require('mongoose');

const CategorySchema  = new mongoose.Schema({

    name: {
        type: String,
        required: true,
    },
});

CategorySchema.set('versionKey', false);

// Export the Movie model
module.exports = mongoose.model('Category', CategorySchema);