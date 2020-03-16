"use strict";

import HttpService from './HttpService';

export default class ConversationService {

    constructor(){
    }

    static baseURL() {return "http://localhost:3000/chats" }

    static getConversations(){
        return new Promise((resolve, reject) => {
            HttpService.get(this.baseURL(), function(data) {
                resolve(data);
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }

    static getConversation(id) {

        return new Promise((resolve, reject) => {
            HttpService.get(`${ConversationService.baseURL()}/conversation/${id}`, function(data) {
                if(data != undefined || Object.keys(data).length !== 0) {
                    resolve(data);
                }
                else {
                    reject('Error while retrieving booking details');
                }
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }

    static createConversation(conversation) {
        var conv = {
            composedMessage: conversation.body
        }
        console.log("conv",conv)
        var recipientId = conversation.recipientId;
        console.log("conv", conv)
        return new Promise((resolve, reject) => {
            HttpService.post(`${ConversationService.baseURL()}/new_conversation/${recipientId}`, conv, function(data) {
                resolve(data);
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }

    static reply(conversation) {
        var conv = {
            composedMessage: conversation.body
        }
        console.log("conv", conv)
        return new Promise((resolve, reject) => {
            HttpService.post(`${ConversationService.baseURL()}/conversation/${conversation.conversationId}`, conv, function(data) {
                resolve(data);
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }



}