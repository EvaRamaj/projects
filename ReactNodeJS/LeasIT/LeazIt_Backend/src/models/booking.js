"use strict";

const mongoose = require('mongoose');

const BookingSchema  = new mongoose.Schema({

    booker_id: {
        type: mongoose.Schema.Types.ObjectId,
        required: true,
        ref: 'User'

    },
    item_id: {
        type: mongoose.Schema.Types.ObjectId,
        required: true,
        ref: 'Item'

    },

    active: {
        type: Boolean,
        default: true
    },

    bookingDate: {
        type: Date,
        required: true,
        //unique: true,
    },

    startDate: {
        type: Date,
        required: true,
        //unique: true,
    },

    endDate: {
        type: Date,
        required: true,
    },

    paymentMethod: {
        type: String,
        //required: true,
        enum: ["card", "cash"]
    },

});

 BookingSchema.set('versionKey', false);

// Export the Movie model
module.exports = mongoose.model('Booking',  BookingSchema);