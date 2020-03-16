"use strict";

const mongoose = require('mongoose');
const User = require('./user');

//bcrypt = require('bcrypt'),
//SALT_WORK_FACTOR = 10;
// Define the user schema

const MessageSchema1  = new mongoose.Schema({

    /*_id: {
        type: Number,
        unique: true,
        // required: true,
    },*/

    receiver_id: {
        type: mongoose.Schema.Types.ObjectId,
        required: true,
        ref: 'User'
        // unique: true,
    },

    sender_id: {
        type: mongoose.Schema.Types.ObjectId,
        required: true,
        ref: 'User'
    },
    read: {
        type: Boolean,
        //required: true,
    },
    text_message: {
        type: String,
        //required: true,
    },
    date: {
        type: Date
    },
    message_time: {
        type: String
    }

});

MessageSchema1.set('versionKey', false);

// Export the Movie model
module.exports = mongoose.model('Message1', MessageSchema1);