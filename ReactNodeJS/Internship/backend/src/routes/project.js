"use strict";

const express        = require('express');
const router         = express.Router();

const ProjectController = require('../controllers/project');

router.post('/search', ProjectController.search);
router.post('/create', ProjectController.create);
router.get('/recommendation/:id', ProjectController.getRecommendation);
router.get('/recommendations', ProjectController.getRecommendationFromES);
router.get('/:id', ProjectController.read);

module.exports = router;