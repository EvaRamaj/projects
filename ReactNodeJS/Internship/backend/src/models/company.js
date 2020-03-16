"use strict";

const mongoose = require('mongoose');

// Define the movie schema

const CompanySchema  = new mongoose.Schema({
    title: {
      type: String,
      required: true
    },
    prefix: {
        type: String,
        enum: ["Mr.", "Ms.","Mrs."]
    },
    firstName: {
        type: String
    },
    lastName: {
        type: String
    },
    address: {
        addressline: String,
        postcode: Number,
        city: String,
        country: String
    },
    phoneNo: String
});

CompanySchema.set('timestamps', true);

// Export the Person model
module.exports = mongoose.model('Company', CompanySchema);
