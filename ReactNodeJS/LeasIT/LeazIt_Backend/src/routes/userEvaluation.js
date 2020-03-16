"use strict";

const express  = require('express');
const router   = express.Router();

const middlewares    = require('../middlewares');
const UserEvaluationController = require('../controllers/userEvaluation');

// --TODO if ADMIN needs it
//router.get('/', UserEvaluationController.list);
// evaluate a user based on their id
router.post('/:Evaluatee_Id', middlewares.checkAuthentication, UserEvaluationController.create);

// --TODO maybe not used
// router.get('/:id', UserEvaluationController.read);
// --TODO update and delete not permited for users
router.put('/:id', /*middlewares.checkAuthentication,*/ UserEvaluationController.update);
// --TODO only admin can do that in order to avoid offensive comments
router.delete('/:id', /*middlewares.checkAuthentication,*/ UserEvaluationController.remove);
// view all evaluations of a specific user -- no authentication needed
router.get('/:UserId', UserEvaluationController.readOtherUserEvaluations);
// view all evaluations depending on users authenticated id
router.get('/', middlewares.checkAuthentication, UserEvaluationController.readOwnEvaluations);

router.get('/user_evaluation/:id', middlewares.checkAuthentication, UserEvaluationController.getUserEvaluation);

module.exports = router;