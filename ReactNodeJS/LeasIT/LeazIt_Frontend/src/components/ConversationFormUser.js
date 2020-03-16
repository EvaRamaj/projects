"use strict";

import React from 'react';
import { Card, Button, FontIcon, TextField } from 'react-md';
import { withRouter } from 'react-router-dom'

import { AlertMessage } from './AlertMessage';
import Page from './Page';
import { Form, FormGroup, Label, Input } from 'reactstrap';


const style = { maxWidth: 500 };


class ConversationFormUser extends React.Component {

    constructor(props) {
        super(props);
        this.state = {
            body: '',
            recipient_username: props.username
        }
        this.handleChangeBody = this.handleChangeBody.bind(this);
        this.handleSubmit = this.handleSubmit.bind(this);
    }

    handleChangeBody(event) {
        this.setState(Object.assign({}, this.state, {body: event.target.value}));
    }


    handleSubmit(event) {
        event.preventDefault();

        let conversation = this.props.categories;
        if(conversation == undefined) {
            conversation = {};
        }

        conversation.body = this.state.body;
        conversation.recipient_username = this.state.recipient_username
        this.props.onSubmit(conversation);
    }

    render() {
        return (
            <Page>
                <Card style={style} className="md-block-centered">
                    <Form onSubmit={this.handleSubmit}>
                        <FormGroup>
                            <Label for="exampleEmail">Write your message!</Label>
                            <Input onChange={this.handleChangeBody} type="textarea" name="comment" id="comment" />
                        </FormGroup>
                        <Button style={{float: "right"}} type="submit" >Submit</Button>
                    </Form>
                </Card>
            </Page>
        );
    }
}

export default withRouter(ConversationFormUser);