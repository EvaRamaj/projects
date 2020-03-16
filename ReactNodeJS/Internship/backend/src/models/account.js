"use strict";

const mongoose = require('mongoose');

// Define the movie schema

const AccountSchema  = new mongoose.Schema({
    username: {
        type: String,
        required: true,
        unique: true
    },
    password: {
        type: String,
        required: true
    },
    userId: {type: mongoose.Schema.Types.ObjectId, ref: 'UserProfile'}
});

AccountSchema.set('timestamps', true);

// Export the Movie model
module.exports = mongoose.model('Account', AccountSchema);
