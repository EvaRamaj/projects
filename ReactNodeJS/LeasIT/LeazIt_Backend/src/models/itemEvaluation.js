"use strict";

const mongoose = require('mongoose');

const ItemEvaluationSchema  = new mongoose.Schema({

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
    item_id: {
        type: mongoose.Schema.Types.ObjectId,
        required: true,
        ref: 'Item'

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

 ItemEvaluationSchema.set('versionKey', false);

// Export the Movie model
module.exports = mongoose.model('ItemEvaluation',  ItemEvaluationSchema);