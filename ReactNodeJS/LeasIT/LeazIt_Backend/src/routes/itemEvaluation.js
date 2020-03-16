"use strict";

const express  = require('express');
const router   = express.Router();

const middlewares    = require('../middlewares');
const ItemEvaluationController = require('../controllers/itemEvaluation');

// --TODO maybe not needed -- only if admin needs it
router.get('/', ItemEvaluationController.list);
// --TODO create an item evaluation only if a user has booked the item already
router.post('/:ItemId', middlewares.checkAuthentication, ItemEvaluationController.create);
// --TODO maybe not needed
// router.get('/:id', ItemEvaluationController.read);
// --TODO maybe not needed to let a user update and delete the evaluation
router.put('/:id', middlewares.checkAuthentication, ItemEvaluationController.update);
router.delete('/:id', /*middlewares.checkAuthentication,*/ ItemEvaluationController.remove);
// gets all evaluations given an itemId
router.get('/:ItemId', ItemEvaluationController.readAnyItemEvaluations);

router.get('/item_evaluation/:id', middlewares.checkAuthentication, ItemEvaluationController.getItemEvaluation);

module.exports = router;