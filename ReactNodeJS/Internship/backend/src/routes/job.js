"use strict";

const express  = require('express');
const router   = express.Router();

const middlewares    = require('../middlewares');
const JobController = require('../controllers/job');


router.get('/', JobController.listAll); // list all jobs
router.post('/create', JobController.create); // create new job
// router.post('/doComment', middlewares.checkAuthentication, BlogController.insertComment); // add comment to blog
// router.get('/featured', BlogController.getFeaturedBlog); // Read featured blog
router.get('/:id', JobController.read); // Read a job by Id
router.put('/:id', JobController.update); // Update a blog by Id
// router.delete('/:id', middlewares.checkAuthentication, BlogController.remove); // Delete a movie by Id

module.exports = router;
