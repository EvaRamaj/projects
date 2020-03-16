"use strict";

const mongoose = require('mongoose');
const Photo = require('./photo');
const Role = require('../role_constants');


///Define the user schema

const UserSchema  = new mongoose.Schema({

    username: {
        type: String,
        required: true,
        unique: true,
    },

    password: {
        type: String,
        required: true,
        unique: true,
    },

    password_salt: {
        type: String
    },

    first_name: {
        type: String,
        //required: true,
        default: ''
    },

    last_name: {
        type: String,
        // required: true,
        default: ''
    },

    email:  {
        type: String,
        required: true,
        unique: true,
    },

    coordinates:{
        lat: Number,
        lng:Number
    },

    address: {
        type: String,
        // road: String,
        // number: String,
        // ZIP: String,
        // city: String,
        // country: String,
        // coordinates: {latitude: Number, longitude:Number},
        // //required: true
        default: ''
    },

    phone: {
        type: String,
        //required: true
        default: ''
    },


    photo: {
        type: String,
        // ref: 'Photo'
        default: ''
    },

    id_document: {
        type: String,
        default: ''
    },

    is_Lessor: {
        type: Boolean,
        //required: true,
        default: false
    },
    role: {
        type: String,
        enum: [Role.ROLE_LESSEE, Role.ROLE_LESSOR, Role.ROLE_ADMIN],
        default: Role.ROLE_LESSEE
    },
    lessor_role_requested: {
        type: Boolean,
        default: false
    }


});
// Export the Movie model
module.exports = mongoose.model('User', UserSchema);