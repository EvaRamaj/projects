"use strict";

const mongoose = require('mongoose');

// Define the movie schema

const UserProfileSchema  = new mongoose.Schema({
    prefix: {
      type: String,
      enum: ["Mr.", "Ms.","Mrs."]
    },
    firstName: {
        type: String,
        required: true
    },
    lastName: {
        type: String,
        required: true
    },
    age: Number,
    address: {
        addressline: String,
        postcode: Number,
        city: String,
        country: String
    },
    gender: {
        type: String,
        enum: ["male", "female"]
    },
    experience:
      {
        description: String,
        compName: String,
        from: Date,
        to: Date
      },
    projects: {
        type: String
    },
    travelPref: {
        type: String,
        enum: ["Yes", "No", "Maybe"]
    },
    matchPref: {
      industry: Number,
      skillset: Number,
      interest: Number
    },
    cv: {
      type: String,
      default: ''
    },
    coverLetter: {
      type: String,
      default: ''
    },
    education: {
        type: String
    },
    admin:{
        type: Boolean
    },
    phoneNo: String
});

UserProfileSchema.set('timestamps', true);

// Export the Movie model
module.exports = mongoose.model('UserProfile', UserProfileSchema);
