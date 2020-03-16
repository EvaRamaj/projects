"use strict";

const mongoose = require('mongoose');
const User = require('./user');
const Category = require('./category');
const Photo = require('./photo');

const ItemSchema  = new mongoose.Schema({

    owner_id: {
        type: mongoose.Schema.Types.ObjectId,
        ref: 'User',
        required: true
    },

    name: {
        type: String,
        required: true,
    },

    condition: {
        type: String,
        required: true,
        enum: ["new", "used", "good", "bad"]
    },

    categories: [{
        type: mongoose.Schema.Types.ObjectId,
        ref: 'Category',
    }],

    description: {
        type: String,
        required: true,
    },

    price:  {
        type: Number,
        required: true,
    },

    pick_up_address: {
        road: String,
        number: String,
        ZIP: String,
        city: String,
        country: String,
        coordinates: {latitude: Number, longitude:Number}
    },

    availability: {
        type: Boolean,
    },

    photos: [{
        type: String,
    }],

});

ItemSchema.set('versionKey', false);

// Export the Movie model
module.exports = mongoose.model('Item', ItemSchema);