"use strict";

const express  = require('express');
const router   = express.Router();

const middlewares    = require('../middlewares');
const ChatController = require('../controllers/conversation');

// View messages to and from authenticated user
router.get('/', middlewares.checkAuthentication, ChatController.getConversations);
// Retrieve single conversation
router.get('/conversation/:conversationId',middlewares.checkAuthentication, ChatController.getConversation);
// Send reply in conversation
router.post('/conversation/:conversationId', middlewares.checkAuthentication, ChatController.sendReply);
// Start new conversation
router.post('/new_conversation/:recipient', middlewares.checkAuthentication, ChatController.newConversation);
// update a message
router.put('/conversation/:conversationId/message/:messageId', middlewares.checkAuthentication, ChatController.updateMessage);
// delete a message
router.delete('/conversation/:conversationId/message/:messageId', middlewares.checkAuthentication, ChatController.deleteMessage);
// delete a conversation
router.delete('/conversation/:conversationId', middlewares.checkAuthentication, ChatController.deleteConversation);

// export router
module.exports = router;