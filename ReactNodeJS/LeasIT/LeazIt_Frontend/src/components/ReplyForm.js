"use strict";

import React from 'react';
import { Card, Button, FontIcon, TextField } from 'react-md';
import { withRouter } from 'react-router-dom'

import { AlertMessage } from './AlertMessage';
import Page from './Page';


const style = { maxWidth: 500 };


class ReplyForm extends React.Component {

    constructor(props) {
        super(props);
        this.state = {
            body: '',
            recipientId: ''
        }
        this.handleChangeBody = this.handleChangeBody.bind(this);
        this.handleChangeRecipientId = this.handleChangeRecipientId.bind(this);
        this.handleSubmit = this.handleSubmit.bind(this);
    }

    handleChangeBody(value) {
        this.setState(Object.assign({}, this.state, {body: value}));
    }

    handleChangeRecipientId(value) {
        this.setState(Object.assign({}, this.state, {recipientId: value}));
    }

    handleSubmit(event) {
        event.preventDefault();

        let conversation = this.props.categories;
        if(conversation == undefined) {
            conversation = {};
        }

        conversation.body = this.state.body;
        conversation.recipientId = this.state.recipientId
        this.props.onSubmit(conversation);
    }

    render() {
        return (
            <Page>
                <Card style={style} className="md-block-centered">
                    <form className="md-grid" onSubmit={this.handleSubmit} onReset={() => this.props.history.goBack()}>
                        <TextField
                            label="body"
                            id="TitleField"
                            type="text"
                            className="md-row"
                            required={true}
                            value={this.state.body}
                            onChange={this.handleChangeBody}
                            errorText="Title is required"/>
                        <TextField
                            label="recipient"
                            id="TitleField"
                            type="text"
                            className="md-row"
                            required={true}
                            value={this.state.recipientId}
                            onChange={this.handleChangeRecipientId}
                            errorText="Title is required"/>
                        <Button id="submit" type="submit"
                                raised primary className="md-cell md-cell--2">Save</Button>
                        <Button id="reset" type="reset" raised secondary className="md-cell md-cell--2">Dismiss</Button>
                        <AlertMessage className="md-row md-full-width" >{this.props.error ? `${this.props.error}` : ''}</AlertMessage>
                    </form>
                </Card>
            </Page>
        );
    }
}

export default withRouter(ReplyForm);