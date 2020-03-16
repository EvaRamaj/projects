"use strict";

const mongoose = require('mongoose');

const UserEvaluationSchema  = new mongoose.Schema({

    /*_id: {
        type: Number,
        unique: true,
        // required: true,
    },*/
    evaluator_id: {
        type: mongoose.Schema.Types.ObjectId,
        required: true,
        ref: 'User'

    },

    evaluatee_id: {
        type: mongoose.Schema.Types.ObjectId,
        required: true,
        ref: 'User'

    },
	rating: {
        type: Number,
        //required: true,
    },

    date: {
        type: Date
    },

    comment: {
        type: String,
        //required: true,
    },

});

 UserEvaluationSchema.set('versionKey', false);

// Export the Movie model
module.exports = mongoose.model('UserEvaluation',  UserEvaluationSchema);