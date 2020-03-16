"use strict";

import React from 'react';

import ReplyForm from './../components/ReplyForm';

import ConversationService from '../services/ConversationService';


export class ReplyFormView extends React.Component {

    constructor(props) {
        super(props);
    }

    componentWillMount(){
            this.setState({
                loading: false,
                message: undefined,
                error: undefined
            });
    }

    reply(message) {
            ConversationService.reply(message).then((data) => {
                this.props.history.push('/my_conversations');
            }).catch((e) => {
                console.error(e);
                this.setState(Object.assign({}, this.state, {error: 'Error while creating item'}));
            });
    }

    render() {
        if (this.state.loading) {
            return (<h2>Loading...</h2>);
        }
        return (<ReplyForm message={this.state.message} onSubmit={(message) => this.reply(message)} error={this.state.error} />);
    }
}
