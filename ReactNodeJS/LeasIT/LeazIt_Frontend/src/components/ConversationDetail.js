"use strict";

import React from 'react';
import { Link } from 'react-router-dom'
import { Card, CardTitle, CardText, Media, MediaOverlay, Grid, Cell, Button, FontIcon } from 'react-md';
import MessageList from './MessageList';
import Page from './Page';

import AuthService from '../services/AuthService';

export class ConversationDetail extends React.Component {

    constructor(props) {
        super(props);

    }

    render() {
        let conversation = this.props.conversation
        return (
            <MessageList data={conversation[0].messages} username={this.props.username} id={this.props.id}/>
        );
    }
}