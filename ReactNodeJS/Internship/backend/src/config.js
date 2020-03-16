"use strict";

//Configuration variables
const port      = process.env.PORT        || '3000';
const mongoURI  = process.env.MONGODB_URI || 'mongodb://0.0.0.0:27017/wayerdb';
const JwtSecret = process.env.JWT_SECRET  || 'causeImmiRocks';

module.exports = {
    port,
    mongoURI,
    JwtSecret,
};
