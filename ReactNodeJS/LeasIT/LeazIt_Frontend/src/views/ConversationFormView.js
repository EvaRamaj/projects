"use strict";

import React from 'react';

import ConversationForm from './../components/ConversationForm';
import ConversationService from '../services/ConversationService';
import UserService from '../services/UserService';



export class ConversationFormView extends React.Component {

    constructor(props) {
        super(props);
    }

    componentWillMount(){
            this.setState({
                loading: false,
                conversation: undefined,
                error: undefined
            });
    }

    createConversation(conversation) {
            UserService.getUserByUsername(conversation.recipient_username).then((data) => {
                conversation.recipientId = data;
                ConversationService.createConversation(conversation).then((data) => {
                    this.props.history.push('/my_conversations');
                }).catch((e) => {
                    console.error(e);
                    this.setState(Object.assign({}, this.state, {error: 'Error while creating item'}));
                })
            }).catch((e) => {
                console.error(e);
                this.setState(Object.assign({}, this.state, {error: 'Error while creating item'}));
        })
    }

    render() {
        if (this.state.loading) {
            return (<h2>Loading...</h2>);
        }
        return (<ConversationForm conversation={this.state.conversation} onSubmit={(conversation) => this.createConversation(conversation)} error={this.state.error} />);
    }
}
